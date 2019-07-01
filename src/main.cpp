#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <ArduinoOTA.h>


#define PI 3.1415926535897932384626433832795

#define SERIALDEBUG
//#define MQTT

#define WIFI_SSID "Lio"
#define WIFI_PASSWORD "l10_wifi"

#define SERIAL_FREQUENCY 10

#define MQTT_SERVER_ADDRESS "192.168.43.197" //Lio - HS
#define MQTT_FREQUENCY 10
#define MQTT_UID 1234

#define ADC_FREQUENCY 5000
#define ADC_1_PIN 36
#define ADC_2_PIN 39
#define FIFO_LENGTH 15
#define RMS_WINDOW 60
#define ADC_DECAY_FREQUENCY 100
#define ADC_DECAY_AMOUNT 0.003

TaskHandle_t Task1, Task2;

long _adc1_value = 0;
long _adc2_value = 0;

float _current = 0.;
float _current_frequency = 0.;
float _current_rms = 0.;
long _current_fifo[FIFO_LENGTH];
long _current_fifo_avg = 0;
long _current_fifo_max = 0;
long _current_fifo_min = 9999;
long _current_fifo_zero_threshold = 0;
long _current_cross = 0;

float _voltage = 0.;
float _voltage_frequency = 0.;
float _voltage_rms = 0.;
long _voltage_fifo[FIFO_LENGTH];
long _voltage_fifo_avg = 0;
long _voltage_fifo_max = 0;
long _voltage_fifo_min = 9999;
long _voltage_fifo_zero_threshold = 0;
long _voltage_cross = 0;

long _voltage_max_discard = 0;
long _voltage_min_discard = 0;
long _voltage_frequency_discard = 0;
long _current_max_discard = 0;
long _current_min_discard = 0;
long _current_frequency_discard = 0;
long _offset_discard = 0;

long _rms_voltage[RMS_WINDOW];
long _rms_current[RMS_WINDOW];


long _offset = 0;
float _phase = 0.;
float _power_factor = 0.;
float _apparent_power = 0.;
float _reactive_power = 0.;
float _real_power = 0.;

long _voltage_last_value = 0;
long _current_last_value = 0;


float _adj_volt_zero = 105;
float _adj_volt_k = 0.92;
float _adj_volt_offset = 8.;
float _adj_cur_zero = -372;
float _adj_cur_k = 1.54;
float _adj_cur_offset = -0.2;

WiFiClient _esp_client;
PubSubClient _mqtt_client(_esp_client);

void setup_wifi() {

    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFiClass::status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    ArduinoOTA.setHostname("instru");

    ArduinoOTA
            .onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH)
                    type = "sketch";
                else // U_SPIFFS
                    type = "filesystem";

                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                Serial.println("Start updating " + type);
            })
            .onEnd([]() {
                Serial.println("\nEnd");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });

    ArduinoOTA.begin();

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

float paylod_to_float(const byte *payload, int lenght) {
    char buffer[20];
    for (int i = 0; i < lenght; i++) {
        buffer[i] = (char)payload[i];
    }
    return atof(buffer);
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    float val = paylod_to_float(payload, length);
    if (*topic == *"sensor/1234/adj/volt_zero") {
        Serial.print("volt_zero: ");
        Serial.println(val);
        _adj_volt_zero = val;
    } else if (*topic == *"sensor/1234/adj/volt_offset") {
        Serial.print("volt_offset: ");
        Serial.println(val);
        _adj_volt_offset = val;
    } else if (*topic == *"sensor/1234/adj/volt_k") {
        _adj_volt_k = val;
    }

    // Switch on the LED if an 1 was received as first character
    if ((char) payload[0] == '1') {
        digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW is the voltage level
        // but actually the LED is on; this is because
        // it is active low on the ESP-01)
    } else {
        digitalWrite(BUILTIN_LED, HIGH); // Turn the LED off by making the voltage HIGH
    }
}

void reconnect() {
    // Loop until we're reconnected
    while (!_mqtt_client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random _mqtt_client ID
        String clientId = "ESP#@-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (_mqtt_client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            _mqtt_client.subscribe("sensor/1234/adj/volt_zero");
            _mqtt_client.subscribe("sensor/1234/adj/volt_offset");
            _mqtt_client.subscribe("sensor/1234/adj/volt_k");
            _mqtt_client.subscribe("sensor/1234/adj/cur_zero");
            _mqtt_client.subscribe("sensor/1234/adj/cur_offset");
            _mqtt_client.subscribe("sensor/1234/adj/cur_k");
        } else {
            Serial.print("failed, rc=");
            Serial.print(_mqtt_client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }

}

void set_float_value(float new_frequency, float *old_frequency, long *counter, long max_counter) {
    if (fabs(double(new_frequency - *old_frequency)) > fabs(*old_frequency * 0.3)) {
        *counter += *counter + 1;
        if (*counter >= max_counter) {
            *old_frequency = new_frequency;
            *counter = 0;
        }
    } else {
        *old_frequency = new_frequency;
        *counter = 0;
    }
}

void set_long_value(long new_val, long *old_val, long *counter, long max_counter) {
    if (abs(new_val - *old_val) > abs(long(*old_val * 0.3))) {
        *counter += *counter + 1;
        if (*counter >= max_counter) {
            *old_val = new_val;
            *counter = 0;
        }
    } else {
        *old_val = new_val;
        *counter = 0;
    }
}

void power_calculation() {
    float voltage = _adj_volt_k * (400.f * 2.f / 4095.f *  (_voltage_rms + _adj_volt_zero)  - 400.f) + _adj_volt_offset;
    float current = _adj_cur_k * (5.f * 2.f / 4095.f * (_current_rms + _adj_cur_zero) - 5.f) + _adj_cur_offset;
    _voltage = voltage;
    _current = current;

    double voltage_period = 1. / _current_frequency;

    double phi = 2 * PI * (float) _offset / voltage_period;
    _power_factor = cos(phi);
    _apparent_power = voltage * current;
    _real_power = cos(phi) * _apparent_power;
    _reactive_power = sin(phi) * _apparent_power;

    float phase = _offset * 360 * _voltage_frequency / 1000000;
    _phase = phase < -180 ? 360 + phase : phase;
}


void serial_loop() {
    power_calculation();

//    Serial.print("corrente max: ");
//    Serial.println(_current);
//    Serial.print(_adc2_value);
//    Serial.print(' ');
//    Serial.print(_voltage_fifo_avg);
//    Serial.print(' ');
//    Serial.print(_voltage_fifo_min);
//    Serial.print(' ');
//    Serial.print(_voltage_fifo_max);
//    Serial.print(' ');
    Serial.print(_voltage_frequency);
    Serial.print(' ');
//    Serial.print(_current_frequency);
    Serial.print(_phase);
    Serial.print(' ');
//    Serial.print(_offset);
//    Serial.print(_current_cross);

//    Serial.print(' ');
//    Serial.print(_adc1_value);
//    Serial.print(' ');
//    Serial.print(_adc2_value);

    Serial.print(_voltage);
//    Serial.print(' ');
//    Serial.print(_current_rms);

    Serial.println();
}

void mqtt_loop() {
    char msg_a[50];
    char msg_v[50];
    char msg_var[50];
    char msg_w[50];
    char msg_vs[50];
    char msg_pf[50];
    char msg_phi[50];

    char msg_a1[50];
    char msg_v1[50];
    char msg_a2[50];
    char msg_v2[50];

    power_calculation();

    sprintf(msg_a, "%ld", (long) (_current_rms * 10000));
    sprintf(msg_v, "%ld", (long) (_voltage_rms * 10000));
    sprintf(msg_var, "%ld", (long) (_reactive_power * 10000));
    sprintf(msg_w, "%ld", (long) (_real_power * 10000));
    sprintf(msg_vs, "%ld", (long) (_apparent_power * 10000));
    sprintf(msg_pf, "%ld", (long) (_power_factor * 10000));
    sprintf(msg_phi, "%ld", (long) (_phase * 10000));

    sprintf(msg_a1, "%ld", (long) (_current_fifo_avg));
    sprintf(msg_v1, "%ld", (long) (_voltage_fifo_avg));
    sprintf(msg_a2, "%ld", (long) (_current*1000));
    sprintf(msg_v2, "%ld", (long) (_voltage*1000));

//    _mqtt_client.publish("board/1234/sensor/1", msg_pf);
//    _mqtt_client.publish("board/1234/sensor/2", msg_w);
//    _mqtt_client.publish("board/1234/sensor/3", msg_var);
//    _mqtt_client.publish("board/1234/sensor/4", msg_vs);
//    _mqtt_client.publish("board/1234/sensor/5", msg_v);
//    _mqtt_client.publish("board/1234/sensor/6", msg_a);
//    _mqtt_client.publish("board/1234/sensor/7", msg_phi);

    _mqtt_client.publish("cal/adc_a", msg_a1);
    _mqtt_client.publish("cal/adc_v", msg_v1);
    _mqtt_client.publish("cal/a", msg_a2);
    _mqtt_client.publish("cal/v", msg_v2);
}

void fifo_loop() {
    long current = 0;
    long voltage = 0;

    for (int i = 0; i < FIFO_LENGTH - 1; i++) {
        current += _current_fifo[i + 1];
        _current_fifo[i] = _current_fifo[i + 1];
        voltage += _voltage_fifo[i + 1];
        _voltage_fifo[i] = _voltage_fifo[i + 1];
    }

    _current_fifo[FIFO_LENGTH - 1] = _adc1_value;
    current += _adc1_value;

    _voltage_fifo[FIFO_LENGTH - 1] = _adc2_value;
    voltage += _adc2_value;

    current /= FIFO_LENGTH;
    voltage /= FIFO_LENGTH;


    if (voltage > _voltage_fifo_max) {
//        _voltage_fifo_max = voltage;
        set_long_value(voltage, &_voltage_fifo_max, &_voltage_max_discard, 100);
    }
    if (current > _current_fifo_max) {
//        _current_fifo_max = current;
        set_long_value(current, &_current_fifo_max, &_current_max_discard, 100);
    }
    if (voltage < _voltage_fifo_min) {
        _voltage_fifo_min = voltage;
    }
    if (current < _current_fifo_min) {
        _current_fifo_min = current;
    }

    _current_fifo_avg = current;
    _voltage_fifo_avg = voltage;
}

//void set_offset(){
//    long offset = _voltage_cross - _current_cross;
//    if (abs(offset - _offset) > abs(long(_offset * 0.3))) {
//        _offset_discard++;
//        if (_offset_discard >= 50) {
//            _offset = offset;
//            _offset_discard = 0;
//        }
//    } else {
//        _offset = offset;
//        _offset_discard = 0;
//    }
//}

void freq_loop() {

    long voltage_mid = (_voltage_fifo_max + _voltage_fifo_min) / 2;
    long current_mid = (_current_fifo_max + _current_fifo_min) / 2;


    if ((_voltage_fifo_avg > voltage_mid + _voltage_fifo_zero_threshold) &&
        (_voltage_last_value < voltage_mid - _voltage_fifo_zero_threshold)) {
//        _voltage_frequency = 1000000.f / (micros() - _voltage_cross);
        float voltage_frequency = 1000000.f / (micros() - _voltage_cross);
        set_float_value(voltage_frequency, &_voltage_frequency, &_voltage_frequency_discard, 50);
        _voltage_cross = micros();



//        Serial.print(_voltage_last_value);
//        Serial.print(" ");
//        Serial.println(_voltage_fifo_avg);
    }

    if ((_current_fifo_avg > current_mid + _current_fifo_zero_threshold) &&
        (_current_last_value < current_mid - _current_fifo_zero_threshold)) {
//        _current_frequency = 1000000.f / (micros() - _current_cross);
        float current_frequency = 1000000.f / (micros() - _current_cross);
        set_float_value(current_frequency, &_current_frequency, &_current_frequency_discard, 50);
        _current_cross = micros();

//        set_offset();
        long offset = 0;
        if (_voltage_cross < _current_cross) {
            offset = _voltage_cross - _current_cross;
        } else {
            offset = _current_cross - _voltage_cross;
        }
        set_long_value(offset, &_offset, &_offset_discard, 50);

    }

    _voltage_last_value = _voltage_fifo_avg;
    _current_last_value = _current_fifo_avg;


}

void adc_loop() {
    _adc1_value = analogRead(ADC_1_PIN);
    _adc2_value = analogRead(ADC_2_PIN);
}

void decay_loop() {
    long voltage_amount = (long) (_voltage_fifo_max * ADC_DECAY_AMOUNT);
    long current_amount = (long) (_current_fifo_max * ADC_DECAY_AMOUNT);

    _voltage_fifo_max -= voltage_amount;
    _voltage_fifo_min += voltage_amount;

    _current_fifo_max -= current_amount;
    _current_fifo_min += current_amount;
}

void rms_loop() {
    //initiate window
    long current_total = 0;
    long voltage_total = 0;
    for (int i = 0; i < RMS_WINDOW - 1; i++) {
        current_total += _rms_current[i + 1] * _rms_current[i + 1];
        _rms_current[i] = _rms_current[i + 1];
        voltage_total += _rms_voltage[i + 1] * _rms_voltage[i + 1];
        _rms_voltage[i] = _rms_voltage[i + 1];
    }
    _rms_current[RMS_WINDOW - 1] = _adc1_value;
    current_total += _current_fifo_avg * _current_fifo_avg;

    _rms_voltage[RMS_WINDOW - 1] = _adc2_value;
    voltage_total += _voltage_fifo_avg * _voltage_fifo_avg;

    _current_rms = sqrt((double) current_total / RMS_WINDOW);
    _voltage_rms = sqrt((double) voltage_total / RMS_WINDOW);
}

void core0_loop(void *parameter) {
    long mqtt_micros = 0;
    long serial_micros = 0;

    long slowest_loop = 0;
    long last_loop = 0;
    pinMode(LED_BUILTIN, OUTPUT);
    setup_wifi();
#ifdef MQTT
    _mqtt_client.setServer(MQTT_SERVER_ADDRESS, 1883);
    _mqtt_client.setCallback(callback);
#endif
    for (;;) {
        long now = micros();
#ifdef MQTT
        if (!_mqtt_client.connected()) {
            reconnect();
        }

        _mqtt_client.loop();
        if (now - mqtt_micros >= 1000000 / MQTT_FREQUENCY) {
            mqtt_loop();
            mqtt_micros = micros();
        }
#endif
        digitalWrite(LED_BUILTIN, HIGH);

#ifdef SERIALDEBUG
        if (now - serial_micros >= 1000000 / SERIAL_FREQUENCY) {
            serial_loop();
            serial_micros = micros();
        }
#endif
        ArduinoOTA.handle();
        vTaskDelay(1);
        last_loop = micros() - now;
        if (last_loop > slowest_loop) {
            slowest_loop = last_loop;
        }

    }
}

void core1_loop(void *parameter) {
    long adc_micros = 0;
    long fifo_micros = 0;
    long freq_micros = 0;
    long decay_micros = 0;
    long rms_micros = 0;

    long slowest_loop = 0;
    for (;;) {
        long now = micros();
        if (now - adc_micros >= 1000000 / ADC_FREQUENCY) {
            adc_loop();
            adc_micros = micros();
        }

        if (now - decay_micros >= 1000000 / ADC_DECAY_FREQUENCY) {
            decay_loop();
            decay_micros = micros();
        }

        if (now - fifo_micros >= 1000000 / (ADC_FREQUENCY * FIFO_LENGTH)) {
            fifo_loop();
            fifo_micros = micros();
        }

        if (now - freq_micros >= 1000000 / (ADC_FREQUENCY)) {
            freq_loop();
            freq_micros = micros();
        }
        float ref_freq = _voltage_frequency > _current_frequency ? _voltage_frequency : _current_frequency;
        if (now - rms_micros >= 1000000 / (ref_freq * RMS_WINDOW / 2)) {
            rms_loop();
            rms_micros = micros();
        }
//        slowest_loop = micros() - now;
//        Serial.print("core1: ");
//        Serial.println(slowest_loop);
    }
}

void setup() {
    pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    Serial.begin(115200);
    analogSetAttenuation(ADC_0db);

    xTaskCreatePinnedToCore(
            core0_loop,
            "core0",
            10000,
            nullptr,
            1,
            &Task1,
            0);
    delay(500);  // needed to start-up task1

    xTaskCreatePinnedToCore(
            core1_loop,
            "core1",
            1000,
            nullptr,
            1,
            &Task2,
            1);
}

void loop() {
    delay(1000);
}
