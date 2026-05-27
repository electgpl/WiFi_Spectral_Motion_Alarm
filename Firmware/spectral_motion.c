#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"

#define TAG "SPECTRAL_MOTION"

#define GPIO_LED            GPIO_NUM_2
#define GPIO_BUZZER         GPIO_NUM_10
#define LED_ON              1
#define LED_OFF             0
#define BUZZER_ON           1
#define BUZZER_OFF          0

#define CALIBRATION_TIME_MS       60000
#define CAL_SCAN_INTERVAL_MS      2000
#define MON_SCAN_INTERVAL_MS      1500
#define ALARM_LATCH_MS            30000
#define ALARM_COOLDOWN_MS         10000

#define MAX_APS                   32
#define RSSI_INVALID              -128
#define CAL_MIN_SAMPLES           5
#define CAL_MAX_NOISE             3.5f
#define MAX_MISSES_BEFORE_REMOVE  8
#define CAL_SETTLE_SCANS          5
#define CAL_QUIET_SCANS           5
#define THRESHOLD_MULT            1.5f
#define THRESHOLD_LOW_MULT        1.3f


#define SLOW_EMA_ALPHA            0.03f
#define FAST_EMA_ALPHA            0.30f
#define SCORE_EMA_ALPHA           0.50f
#define M_OF_N_WINDOW             5
#define M_OF_N_REQUIRED           3
#define RATE_CHANGE_WEIGHT        0.25f
#define DEVIATION_WEIGHT          0.45f
#define DIVERGENCE_WEIGHT         0.10f

#define CSI_WEIGHT                0.30f
#define CSI_PACKET_WEIGHT         0.10f
#define CSI_NUM_SC                64
#define CSI_ACCUM_ALPHA           0.15f
#define CSI_BASELINE_ALPHA        0.05f

typedef struct {
    float acc_real[CSI_NUM_SC];
    float acc_imag[CSI_NUM_SC];
    float mag2_baseline[CSI_NUM_SC];
    float mag2_baseline_slow[CSI_NUM_SC];
    int packet_count;
    float csi_deviation;
    float csi_profile_dev;
    float csi_mean_baseline;
    float csi_mean_baseline_slow;
    float csi_score_contribution;
} csi_data_t;

typedef struct {
    uint8_t bssid[6];
    char bssid_str[18];
    uint8_t channel;
    float cal_samples[30];
    int cal_count;
    float median;
    float noise_floor;
    float slow_baseline;
    float fast_baseline;
    float current_rssi;
    float prev_rssi;
    float last_diff;
    int misses;
    bool active;
    bool seen;
    csi_data_t csi;
} ap_data_t;

static ap_data_t s_aps[MAX_APS];
static int s_ap_count = 0;
static bool s_calibrated = false;
static float s_raw_score = 0.0f;
static float s_smooth_score = 0.0f;
static float s_alarm_threshold_high = 999.0f;
static float s_alarm_threshold_low = 999.0f;
static float s_quiet_baseline = 0.0f;
static int s_quiet_scans_done = 0;
static int s_alarm_history[M_OF_N_WINDOW];
static int s_alarm_idx = 0;
static bool s_alarm_active = false;
static int s_cal_scans = 0;
static uint64_t s_alarm_trigger_time = 0;
static uint64_t s_start_ms = 0;
static volatile bool s_led_blink_enabled = false;
static esp_timer_handle_t s_led_timer = NULL;
static bool s_wifi_connected = false;

static void send_status(const char *msg);
static int find_ap_index(const uint8_t *bssid);

static void led_timer_cb(void *arg) {
    static bool state = false;
    if (s_led_blink_enabled) {
        state = !state;
        gpio_set_level(GPIO_LED, state ? LED_ON : LED_OFF);
    } else {
        gpio_set_level(GPIO_LED, !s_alarm_active ? LED_ON : LED_OFF);
    }
}

static void gpio_init(void) {
    gpio_reset_pin(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED, LED_OFF);
    gpio_reset_pin(GPIO_BUZZER);
    gpio_set_direction(GPIO_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_BUZZER, BUZZER_OFF);
    const esp_timer_create_args_t targs = { .callback = &led_timer_cb, .name = "led_timer" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_led_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_led_timer, 250000));
}

static void buzzer_set(bool on) {
    gpio_set_level(GPIO_BUZZER, on ? BUZZER_ON : BUZZER_OFF);
}

static int find_ap_index(const uint8_t *bssid) {
    for (int i = 0; i < s_ap_count; i++)
        if (memcmp(s_aps[i].bssid, bssid, 6) == 0) return i;
    return -1;
}

static void bssid_to_str(const uint8_t *bssid, char *str) {
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}

static int sort_by_noise(const void *a, const void *b) {
    const ap_data_t *pa = (const ap_data_t *)a;
    const ap_data_t *pb = (const ap_data_t *)b;
    int aa = pa->active ? 0 : 1;
    int bb = pb->active ? 0 : 1;
    if (aa != bb) return aa - bb;
    if (pa->noise_floor < pb->noise_floor) return -1;
    return 1;
}

static float median(float *arr, int n) {
    float tmp[30];
    memcpy(tmp, arr, n * sizeof(float));
    for (int i = 0; i < n - 1; i++)
        for (int j = i + 1; j < n; j++)
            if (tmp[i] > tmp[j]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
    return tmp[n / 2];
}

static float mad(float *arr, int n, float med) {
    float dev[30];
    for (int i = 0; i < n; i++) dev[i] = fabsf(arr[i] - med);
    return median(dev, n);
}

static void send_cal_progress(int pct, int aps) {
    printf("{\"t\":\"cal\",\"pct\":%d,\"aps\":%d}\n", pct, aps);
}

static void send_state(void) {
    printf("{\"t\":\"state\",\"aps\":%d,\"score\":%.1f,\"alarm\":%s,\"th_high\":%.1f,\"th_low\":%.1f}\n",
           s_ap_count, (double)s_smooth_score,
           s_alarm_active ? "true" : "false",
           (double)s_alarm_threshold_high, (double)s_alarm_threshold_low);
}

static void send_ap_status(int idx) {
    ap_data_t *ap = &s_aps[idx];
    printf("{\"t\":\"ap\",\"bssid\":\"%s\",\"rssi\":%d,\"baseline\":%.1f,\"noise\":%.1f,\"diff\":%.2f,\"misses\":%d,\"csi_score\":%.2f,\"csi_pkt\":%d}\n",
           ap->bssid_str,
           ap->seen ? (int)ap->current_rssi : RSSI_INVALID,
           (double)ap->slow_baseline, (double)ap->noise_floor,
           (double)ap->last_diff, ap->misses,
           (double)ap->csi.csi_score_contribution,
           ap->csi.packet_count);
}

static void send_alarm(bool active, const char *reason) {
    printf("{\"t\":\"alarm\",\"state\":%s,\"score\":%.1f,\"reason\":\"%s\"}\n",
           active ? "true" : "false", (double)s_smooth_score, reason);
}

static void send_status(const char *msg) {
    printf("{\"t\":\"status\",\"msg\":\"%s\"}\n", msg);
}

static esp_err_t do_wifi_scan(wifi_ap_record_t *records, uint16_t *count) {
    wifi_scan_config_t sc = {
        .ssid = NULL, .bssid = NULL, .channel = 0,
        .show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 150, .scan_time.active.max = 400,
    };
    esp_err_t ret = esp_wifi_scan_start(&sc, true);
    if (ret != ESP_OK) return ret;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_scan_get_ap_records(count, records));
    return ESP_OK;
}

static void do_calibration_scan(void) {
    wifi_ap_record_t records[MAX_APS];
    uint16_t count = MAX_APS;
    if (do_wifi_scan(records, &count) != ESP_OK) return;
    if (count == 0) return;
    for (int i = 0; i < count; i++) {
        int rssi = records[i].rssi;
        if (rssi < -100 || rssi > -20) continue;
        int idx = find_ap_index(records[i].bssid);
        if (idx < 0) {
            if (s_ap_count >= MAX_APS) continue;
            idx = s_ap_count++;
            memcpy(s_aps[idx].bssid, records[i].bssid, 6);
            bssid_to_str(records[i].bssid, s_aps[idx].bssid_str);
            s_aps[idx].channel = records[i].primary;
            s_aps[idx].cal_count = 0;
            s_aps[idx].active = false;
            memset(&s_aps[idx].csi, 0, sizeof(csi_data_t));
        }
        if (s_aps[idx].cal_count < 30)
            s_aps[idx].cal_samples[s_aps[idx].cal_count++] = (float)rssi;
    }
    s_cal_scans++;
}

static void finish_calibration(void) {
    int kept = 0;
    for (int i = 0; i < s_ap_count; i++) {
        ap_data_t *ap = &s_aps[i];
        if (ap->cal_count < CAL_MIN_SAMPLES) { ap->active = false; continue; }
        ap->median = median(ap->cal_samples, ap->cal_count);
        float m = mad(ap->cal_samples, ap->cal_count, ap->median);
        ap->noise_floor = (m < 1.0f) ? 1.0f : m;
        if (ap->noise_floor > CAL_MAX_NOISE) { ap->active = false; continue; }
        ap->active = true;
        ap->slow_baseline = ap->median;
        ap->fast_baseline = ap->median;
        ap->current_rssi = ap->median;
        ap->prev_rssi = ap->median;
        ap->misses = 0;
        ap->csi.csi_deviation = 0;
        ap->csi.csi_profile_dev = 0;
        ap->csi.csi_score_contribution = 0;
        for (int s = 0; s < CSI_NUM_SC; s++) {
            ap->csi.mag2_baseline[s] = 0;
            ap->csi.mag2_baseline_slow[s] = 0;
        }
        ap->csi.csi_mean_baseline = 0;
        ap->csi.csi_mean_baseline_slow = 0;
        kept++;
    }
    qsort(s_aps, s_ap_count, sizeof(ap_data_t), sort_by_noise);
    int best_ch = 1, best_ch_count = 0;
    int ch_counts[14] = {0};
    for (int i = 0; i < s_ap_count; i++) {
        if (!s_aps[i].active) continue;
        int ch = s_aps[i].channel;
        if (ch >= 1 && ch <= 13) {
            ch_counts[ch]++;
            if (ch_counts[ch] > best_ch_count) {
                best_ch_count = ch_counts[ch];
                best_ch = ch;
            }
        }
    }
    esp_wifi_set_channel(best_ch, WIFI_SECOND_CHAN_NONE);
    ESP_LOGI(TAG, "CSI home channel set to %d (%d APs)", best_ch, best_ch_count);

    s_calibrated = true;
    if (kept == 0)
        send_status("ERROR: No stable APs found for calibration");
    else {
        char msg[64];
        sprintf(msg, "Calibration OK: %d/%d stable APs, CSI ch=%d", kept, s_cal_scans, best_ch);
        send_status(msg);
    }
}

static void csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    int idx = find_ap_index(info->mac);
    if (idx < 0) return;
    ap_data_t *ap = &s_aps[idx];
    if (!ap->active) return;

    int offset = info->first_word_invalid ? 4 : 0;
    int valid_len = info->len - offset;
    int num_sc = valid_len / 2;
    if (num_sc > CSI_NUM_SC) num_sc = CSI_NUM_SC;
    if (num_sc < 1) return;

    float mean_mag2 = 0;

    for (int i = 0; i < num_sc; i++) {
        int pos = offset + i * 2;
        float real = (float)info->buf[pos];
        float imag = (float)info->buf[pos + 1];
        float mag2 = real * real + imag * imag;
        ap->csi.acc_real[i] += real;
        ap->csi.acc_imag[i] += imag;
        ap->csi.mag2_baseline[i] += CSI_ACCUM_ALPHA * (mag2 - ap->csi.mag2_baseline[i]);
        mean_mag2 += mag2;
    }
    mean_mag2 /= (float)num_sc;

    ap->csi.csi_mean_baseline += CSI_ACCUM_ALPHA * (mean_mag2 - ap->csi.csi_mean_baseline);
    ap->csi.packet_count++;
}

static void process_csi_data(ap_data_t *ap) {
    if (ap->csi.packet_count < 1) {
        ap->csi.csi_deviation = 0;
        ap->csi.csi_profile_dev = 0;
        return;
    }

    float sc_dev_sum = 0;
    int count = 0;

    for (int i = 0; i < CSI_NUM_SC; i++) {
        float fast = ap->csi.mag2_baseline[i];
        float slow = ap->csi.mag2_baseline_slow[i];

        if (slow > 0) {
            float dev = fabsf(fast - slow);
            float rel_dev = dev / (slow + 1.0f);
            sc_dev_sum += rel_dev;
            count++;
        }

        if (ap->csi.packet_count >= 2) {
            ap->csi.mag2_baseline_slow[i] += CSI_BASELINE_ALPHA * (fast - slow);
        } else {
            ap->csi.mag2_baseline_slow[i] = fast;
        }
    }

    float fast_mean = ap->csi.csi_mean_baseline;
    float slow_mean = ap->csi.csi_mean_baseline_slow;

    if (slow_mean > 0) {
        ap->csi.csi_deviation = fabsf(fast_mean - slow_mean) / (slow_mean + 1.0f);
    } else {
        ap->csi.csi_deviation = 0;
    }

    if (slow_mean > 0) {
        ap->csi.csi_mean_baseline_slow += CSI_BASELINE_ALPHA * (fast_mean - slow_mean);
    } else {
        ap->csi.csi_mean_baseline_slow = fast_mean;
    }

    ap->csi.csi_profile_dev = (count > 0) ? (sc_dev_sum / (float)count) : 0;
    ap->csi.csi_score_contribution = (ap->csi.csi_deviation * 0.6f + ap->csi.csi_profile_dev * 0.4f);
    if (ap->csi.csi_score_contribution < 0) ap->csi.csi_score_contribution = 0;

    memset(ap->csi.acc_real, 0, sizeof(ap->csi.acc_real));
    memset(ap->csi.acc_imag, 0, sizeof(ap->csi.acc_imag));
}

static void csi_init(void) {
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
    };
    esp_err_t ret;
    ret = esp_wifi_set_csi_config(&csi_config);
    if (ret != ESP_OK) { ESP_LOGW(TAG, "CSI config failed: %d", ret); return; }
    ret = esp_wifi_set_csi_rx_cb(&csi_rx_cb, NULL);
    if (ret != ESP_OK) { ESP_LOGW(TAG, "CSI register failed: %d", ret); return; }
    ret = esp_wifi_set_csi(true);
    if (ret != ESP_OK) { ESP_LOGW(TAG, "CSI enable failed: %d", ret); send_status("CSI not available, running RSSI only"); return; }
    ESP_LOGI(TAG, "CSI initialized: HT20, 64 subcarriers");
    send_status("CSI enabled on all tracked APs");
}

static float compute_score(void) {
    float score = 0.0f;
    float csi_score = 0.0f;
    int high_count = 0;

    for (int i = 0; i < s_ap_count; i++) {
        ap_data_t *ap = &s_aps[i];
        if (!ap->active) continue;

        float deviation = 0.0f;
        float rate = 0.0f;
        float divergence = 0.0f;

        if (ap->seen) {
            float rssi = ap->current_rssi;
            deviation = fabsf(rssi - ap->slow_baseline) / ap->noise_floor;
            rate = fabsf(rssi - ap->prev_rssi) / ap->noise_floor;
            divergence = fabsf(ap->fast_baseline - ap->slow_baseline) / ap->noise_floor;
        } else {
            deviation = 2.5f;
        }

        ap->last_diff = deviation;

        float rssi_total = deviation * DEVIATION_WEIGHT
                         + rate * RATE_CHANGE_WEIGHT
                         + divergence * DIVERGENCE_WEIGHT;

        if (ap->csi.packet_count >= 1) {
            csi_score += ap->csi.csi_score_contribution * CSI_WEIGHT;
        }

        score += rssi_total;

        if (deviation > 2.0f) high_count++;
    }

    float bonus = (high_count > 2) ? (high_count - 2) * 0.5f : 0.0f;
    score += bonus + csi_score;

    return score;
}

static void do_monitor_scan(void) {
    static uint64_t last_reconnect_ms = 0;
    if (strlen(CONFIG_WIFI_SSID) > 0 && !s_wifi_connected) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - last_reconnect_ms > 10000) {
            last_reconnect_ms = now;
            esp_wifi_connect();
        }
    }

    wifi_ap_record_t records[MAX_APS];
    uint16_t count = MAX_APS;
    if (do_wifi_scan(records, &count) != ESP_OK) return;

    for (int i = 0; i < s_ap_count; i++) s_aps[i].seen = false;

    for (int i = 0; i < count; i++) {
        int idx = find_ap_index(records[i].bssid);
        if (idx < 0 || !s_aps[idx].active) continue;
        ap_data_t *ap = &s_aps[idx];
        ap->seen = true;
        ap->misses = 0;
        float rssi = (float)records[i].rssi;
        ap->prev_rssi = ap->current_rssi;
        ap->current_rssi = rssi;
        ap->slow_baseline += SLOW_EMA_ALPHA * (rssi - ap->slow_baseline);
        ap->fast_baseline += FAST_EMA_ALPHA * (rssi - ap->fast_baseline);
    }

    for (int i = 0; i < s_ap_count; i++) {
        if (s_aps[i].active) {
            process_csi_data(&s_aps[i]);
        }
    }

    for (int i = 0; i < s_ap_count; i++) {
        if (!s_aps[i].active) continue;
        if (!s_aps[i].seen) {
            s_aps[i].misses++;
            if (s_aps[i].misses >= MAX_MISSES_BEFORE_REMOVE) {
                s_aps[i].active = false;
            }
        }
    }

    s_raw_score = compute_score();
    s_smooth_score += SCORE_EMA_ALPHA * (s_raw_score - s_smooth_score);

    uint64_t now = esp_timer_get_time() / 1000;
    bool in_cooldown = (now < s_alarm_trigger_time + ALARM_COOLDOWN_MS);

    bool reading_triggered = (s_smooth_score > s_alarm_threshold_high) && !in_cooldown;

    s_alarm_history[s_alarm_idx] = reading_triggered ? 1 : 0;
    s_alarm_idx = (s_alarm_idx + 1) % M_OF_N_WINDOW;

    int trigger_count = 0;
    for (int i = 0; i < M_OF_N_WINDOW; i++)
        trigger_count += s_alarm_history[i];

    bool should_alarm = (trigger_count >= M_OF_N_REQUIRED);

    if (should_alarm && !s_alarm_active) {
        s_alarm_active = true;
        s_alarm_trigger_time = now;
        buzzer_set(true);
        send_alarm(true, "Movement detected");
    }

    if (s_alarm_active) {
        uint64_t alarm_elapsed = now - s_alarm_trigger_time;
        if (alarm_elapsed > ALARM_LATCH_MS && s_smooth_score < s_alarm_threshold_low) {
            s_alarm_active = false;
            buzzer_set(false);
            send_alarm(false, "Environment normalized");
        }
    }

    send_state();

    int sent = 0;
    for (int i = 0; i < s_ap_count && sent < 8; i++) {
        if (s_aps[i].active) {
            send_ap_status(i);
            s_aps[i].csi.packet_count = 0;
            sent++;
        }
    }
}

static void spectral_motion_task(void *pvParams) {
    send_status("WiFi Spectral Motion Detector v3.2-CSI");
    send_status("System started. Stay still during calibration...");

    s_start_ms = esp_timer_get_time() / 1000;

    while (1) {
        if (!s_calibrated) {
            uint64_t now = esp_timer_get_time() / 1000;
            uint64_t elapsed = now - s_start_ms;

            if (elapsed < (uint64_t)CALIBRATION_TIME_MS) {
                do_calibration_scan();
                int pct = (int)(elapsed * 100 / CALIBRATION_TIME_MS);
                if (pct > 100) pct = 100;
                send_cal_progress(pct, s_ap_count);
                s_led_blink_enabled = true;
                vTaskDelay(pdMS_TO_TICKS(CAL_SCAN_INTERVAL_MS));
            } else {
                finish_calibration();
                send_cal_progress(100, s_ap_count);
                send_status("Calibration complete. CSI enabled. Monitoring started.");
                s_quiet_scans_done = 0;
                s_quiet_baseline = 0.0f;
                for (int i = 0; i < M_OF_N_WINDOW; i++) s_alarm_history[i] = 0;
                s_alarm_idx = 0;
                s_alarm_active = false;
                s_led_blink_enabled = false;
            }
        } else {
            do_monitor_scan();
            if (s_quiet_scans_done < CAL_SETTLE_SCANS + CAL_QUIET_SCANS) {
                s_quiet_scans_done++;
                if (s_quiet_scans_done >= CAL_SETTLE_SCANS) {
                    s_quiet_baseline += s_smooth_score;
                }
                if (s_quiet_scans_done >= CAL_SETTLE_SCANS + CAL_QUIET_SCANS) {
                    s_quiet_baseline /= CAL_QUIET_SCANS;
                    s_alarm_threshold_high = s_quiet_baseline * THRESHOLD_MULT;
                    s_alarm_threshold_low = s_quiet_baseline * THRESHOLD_LOW_MULT;
                    if (s_alarm_threshold_high < 2.0f) s_alarm_threshold_high = 2.0f;
                    if (s_alarm_threshold_low < 1.0f) s_alarm_threshold_low = 1.0f;
                    char msg[80];
                    sprintf(msg, "Adaptive threshold: quiet=%.1f high=%.1f low=%.1f",
                            s_quiet_baseline, s_alarm_threshold_high, s_alarm_threshold_low);
                    send_status(msg);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(MON_SCAN_INTERVAL_MS));
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        s_wifi_connected = true;
        ESP_LOGI(TAG, "Connected to AP");
        send_status("Connected to Wi-Fi AP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from AP, reason=%d", d->reason);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

static void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    wifi_config_t wc = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
    wifi_country_t country = { .cc = "US", .schan = 1, .nchan = 11 };
    esp_wifi_set_country(&country);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (strlen(CONFIG_WIFI_SSID) > 0) {
        ESP_LOGI(TAG, "Wi-Fi association configured for SSID: %s", CONFIG_WIFI_SSID);
        send_status("Wi-Fi association configured");
    } else {
        ESP_LOGI(TAG, "No Wi-Fi SSID configured, promiscuous-only mode");
        send_status("No Wi-Fi credentials, promiscuous-only mode");
    }
    ESP_LOGI(TAG, "WiFi STA+promiscuous mode started");
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_init();
    wifi_init();
    csi_init();
    send_status("ESP32-S3 Spectral Motion v3.2-CSI");

    xTaskCreatePinnedToCore(spectral_motion_task, "spectral_motion",
                            8192, NULL, 5, NULL, tskNO_AFFINITY);
}
