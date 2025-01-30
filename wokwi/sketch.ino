#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <Arduino.h>
#include <ArduinoJson.h>



using namespace ArduinoJson;


#define DHTPIN 4         // Pin pour le DHT22
#define PIRPIN 2         // Pin pour le PIR
#define LDRPIN 32        // Pin pour le LDR (capteur de luminosité)
#define LEDPIN_R 25      // Pin pour la LED rouge (RGB)
#define LEDPIN_G 27      // Pin pour la LED verte (RGB)
#define LEDPIN_B 26      // Pin pour la LED bleue (RGB)
 
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Initialiser l'écran LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adresse I2C et dimensions

/////////////////////////////////////////////////
/////////////////////////////////////////////////

char topic_all[] = "all";
char topic_group[20];
char topic_self[20];
char topic_focus[] = "focus";


const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi(void);

void callback(char* topic, byte* payload, unsigned int length);

void reconnect(void);

/////////////////////////////////////////////////
/////////////////////////////////////////////////
void stats_to_json(char *out, size_t out_size);

#define MYDBG(expr) \
	{\
    	Serial.print("\nat line: ");\
    	Serial.println(__LINE__);\
		Serial.println(#expr);\
    	Serial.println(expr);\
	}



class state_t {

public:

	uint32_t gid = 1;
	uint32_t id = 1;

	uint8_t led_wattage = 50;

    bool led_broken = false;

	bool led_intensity_is_manual = false;
	float led_intensity_value = 0;

    uint8_t led_color[3] = {255, 255, 255}; // rgb

    uint32_t movement_last_detection = 0;
    uint32_t movement_time_on_after_detection_ms = 1000;
    bool movement_detected = false;
    bool movement_activation_enabled = true;

    uint32_t light_sensor_value = 0;
    uint32_t light_sensor_activation_value = 100;
    bool light_sensor_activation_enabled = true;

    bool camera_on = false;

	float temp = 0;
	float humidity = 0;

	bool display_on = false;

    bool publish_changes = false;

	void to_json_string(char *out, size_t out_size)
	{
		JsonDocument doc;

		doc["gid"] = this->gid;
		doc["id"] = this->id;

		doc["fail"] = this->led_broken;

		doc["grad"] = this->led_intensity_value;

		doc["mov"] = this->movement_detected;

		doc["lum"] = this->light_sensor_value;

		doc["temp"] = this->temp;
		doc["hum"] = this->humidity;

		serializeJson(doc, out, out_size);
	}
};


static state_t current_state;
static state_t next_state;

static bool made_publishable_changes = true;

// this function will be called by the mqtt pubsubclient callback
static void callback_helper_cmd(const char *request_str)
{
	MYDBG(request_str);

	JsonDocument doc;
	deserializeJson(doc, request_str);

	JsonObject obj = doc.as<JsonObject>();

	const char *gradMode = obj["gradMode"];
	// FIXME: do correct error handling
	if( gradMode == NULL )
		return;

	next_state.led_intensity_is_manual = ( 
		strcmp(gradMode, "manual") == 0 
	);

	if( next_state.led_intensity_is_manual )
		next_state.led_intensity_value = obj["grad"];


	const char *col = obj["col"];
	if( strcmp(col, "white") == 0 )
	{
		next_state.led_color[0] = 255;
		next_state.led_color[1] = 255;
		next_state.led_color[2] = 255;
	}
	else if( strcmp(col, "yellow") == 0 )
	{
		next_state.led_color[0] = 255;
		next_state.led_color[1] = 255;
		next_state.led_color[2] = 0;
	}
	else // red
	{
		next_state.led_color[0] = 255;
		next_state.led_color[1] = 0;
		next_state.led_color[2] = 0;
	}

	next_state.camera_on = obj["cam"];
	next_state.display_on = obj["aff"];
}

static void callback_helper_focus(const char *request_str)
{
	MYDBG(request_str);

	char zone_str[20];
	sprintf(zone_str, "Zone %d", next_state.gid);

	if( strcmp(request_str, zone_str) == 0 )
		next_state.publish_changes = true;
	else
		next_state.publish_changes = false;
}


// read values from the sensors
// and change next_state
static void update_next_state_sensor_values(void)
{
	// Lire l'humidité et la température du DHT22
	next_state.humidity = dht.readHumidity();
	next_state.temp = dht.readTemperature();

	// Lire la valeur du LDR (lumière ambiante)
	// Normaliser la valeur du LDR pour qu'elle soit entre 0 et 255
	int ldrValue = analogRead(LDRPIN);
	next_state.light_sensor_value = 255 - map(ldrValue, 0, 4095, 0, 255);


	// Lit l'état du capteur PIR
	int pirState = digitalRead(PIRPIN);
	if( pirState == HIGH ) {
		next_state.movement_detected = true;
		next_state.movement_last_detection = millis();
	}
	else {
		next_state.movement_detected = false;
	}
}

// make changes in next_state
static void update_next_state(void)
{
	update_next_state_sensor_values();

	bool bad_brightness = (
		next_state.light_sensor_value <
		next_state.light_sensor_activation_value
	);

	bool recent_movement = next_state.movement_detected;

	if( next_state.led_broken )
		next_state.led_intensity_value = 0;
	else
		if( ! next_state.led_intensity_is_manual )
		{
			if( bad_brightness )
			{
				if( next_state.camera_on )
					next_state.led_intensity_value = 100;
				else
					if( recent_movement )
						next_state.led_intensity_value = 50;
					else
						next_state.led_intensity_value = 0;
			}
			else
				next_state.led_intensity_value = 0;
		}
}

// may send intructions to hardware to mactch next_state
// may set made_publishable_changes to true
static void update_hardware(void)
{

	if(
		current_state.led_intensity_value != next_state.led_intensity_value ||
		memcmp(
			current_state.led_color,
			next_state.led_color,
			sizeof(current_state.led_color)
		)
	) {
		if( ! next_state.led_intensity_is_manual )
			made_publishable_changes = true;

		float I = next_state.led_intensity_value / 100;
		analogWrite(LEDPIN_R, I*next_state.led_color[0]);
		analogWrite(LEDPIN_G, I*next_state.led_color[1]);
		analogWrite(LEDPIN_B, I*next_state.led_color[2]);
	}

	// check if sensor values changed
	if( 
		current_state.temp != next_state.temp ||
		current_state.humidity != next_state.humidity ||
		current_state.light_sensor_value != next_state.light_sensor_value ||
		current_state.movement_detected != next_state.movement_detected
	)
		made_publishable_changes = true;
	

	// Afficher l'humidité et la température sur l'écran LCD
	if(
		current_state.display_on != next_state.display_on ||
		current_state.temp != next_state.temp ||
        current_state.humidity != next_state.humidity
	) {

		lcd.clear();
		if( next_state.display_on )
		{
			lcd.setCursor(0, 0);
			lcd.print("Temp: ");
			lcd.print(next_state.temp);
			lcd.print("C");

			lcd.setCursor(0, 1);
			lcd.print("Hum: ");
			lcd.print(next_state.humidity);
			lcd.print("%");
		}
	}
}

/////////////////////////////////////////////////

class stat_point_t {
	public:
		float time_led_on_ms = 0;
		uint8_t led_broken = 0;
		float led_intensity = 0;
		float power_Wh = 0;

		float light_intensity = 0;
		float temp = 0;
		float humidity = 0;

		// used to compute the averages
		uint32_t total_time_ms = 0;
};

uint32_t last_stat_point_creation_time;
uint32_t last_stat_point_last_time;


std::vector<stat_point_t> stats;

void update_stats(void)
{
	uint32_t current_time = millis();

	if( stats.size() == 0 )
	{
		stats.push_back(stat_point_t());
		last_stat_point_creation_time = current_time;
		last_stat_point_last_time = current_time;
	}

	uint32_t dt = current_time - last_stat_point_last_time;
	last_stat_point_last_time = current_time;

	stats.back().total_time_ms += dt;

	// stat_point_t::time_led_on_ms
	if( current_state.led_intensity_value > 0 )
		stats.back().time_led_on_ms += dt;

	// stat_point_t::led_broken
	if( current_state.led_broken )
		stats.back().led_broken = 1;

	// stat_point_t::led_intensity
	stats.back().led_intensity += current_state.led_intensity_value * dt;

	// stat_point_t::power_Wh
	stats.back().power_Wh += 
		current_state.led_intensity_value * current_state.led_wattage * dt / 1000 / 3600;

	// stat_point_t::light_intensity
	stats.back().light_intensity += current_state.light_sensor_value * dt;

	// stat_point_t::temp
	stats.back().temp += current_state.temp * dt;

	// stat_point_t::humidity
	stats.back().humidity += current_state.humidity * dt;

	if( current_time - last_stat_point_creation_time >= 60*1000 )
	{
		if( stats.size() == 2 )
		{
			// send stats_to_json to server
			char *buf = (char *)calloc(5000, 1);
			stats_to_json(buf, 5000);
			client.publish("report", buf);
			MYDBG(buf);
			free(buf);

			stats.clear();
		}
		else
		{
			stats.push_back(stat_point_t());
			last_stat_point_creation_time = current_time;
			last_stat_point_last_time = current_time;
		}
	}
}

void stats_to_json(char *out, size_t out_size)
{
	JsonDocument doc;

	doc["gid"] = current_state.gid;
	doc["id"] = current_state.id;

	JsonArray time_led_on_ms = doc["time_led_on_ms"].to<JsonArray>();
	JsonArray led_broken = doc["led_broken"].to<JsonArray>();
	JsonArray led_intensity = doc["led_intensity"].to<JsonArray>();
	JsonArray power_Wh = doc["power_Wh"].to<JsonArray>();

	JsonArray light_intensity = doc["light_intensity"].to<JsonArray>();
	JsonArray temp = doc["temp"].to<JsonArray>();
	JsonArray humidity = doc["humidity"].to<JsonArray>();


	for( uint32_t idx = 0 ; idx < stats.size() ; idx++ )
	{
		time_led_on_ms[idx] = stats[idx].time_led_on_ms;
		led_broken[idx] = stats[idx].led_broken;
		led_intensity[idx] = stats[idx].led_intensity / stats[idx].total_time_ms;
		power_Wh[idx] = stats[idx].power_Wh;

		light_intensity[idx] = stats[idx].light_intensity / stats[idx].total_time_ms;
		temp[idx] = stats[idx].temp / stats[idx].total_time_ms;
		humidity[idx] = stats[idx].humidity / stats[idx].total_time_ms;
	}

	serializeJson(doc, out, out_size);
}

/////////////////////////////////////////////////


void loop(void)
{
	if ( ! client.connected() )
		reconnect();
	client.loop();


	update_stats();

	update_next_state();
	update_hardware();

	if( made_publishable_changes &&
		( current_state.publish_changes || next_state.publish_changes )
	)
	{
		// send next_state.to_json_string to server
		char *buf = (char *)calloc(5000, 1);
		next_state.to_json_string(buf, 5000);
		MYDBG(buf);
		client.publish(topic_self, buf);
		free(buf);
	}

	current_state = next_state;
	made_publishable_changes = false;


	delay(1000);
}

void setup(void)
{
	sprintf(topic_group, "group%d", current_state.gid);
	sprintf(topic_self, "group%d/lamp%d", current_state.gid, current_state.id);

	setup_wifi();
	client.setServer(mqtt_server, 1883);
	client.setCallback(callback);

	// Initialiser les pins
	pinMode(PIRPIN, INPUT);
	pinMode(LDRPIN, INPUT);
	pinMode(LEDPIN_R, OUTPUT);
	pinMode(LEDPIN_G, OUTPUT);
	pinMode(LEDPIN_B, OUTPUT);

	// Démarrer la communication série
	Serial.begin(115200);

	// Démarrer le capteur DHT
	dht.begin();

	// Démarrer l'écran LCD et initialiser I2C
	Wire.begin(21, 22); // Broches SDA et SCL pour ESP32
	lcd.begin(16, 2);
	lcd.setBacklight(1); // Rétroéclairage activé
	lcd.print("Initialisation...");
	delay(2000);  // Attendre 2 secondes
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////


void setup_wifi() {
	delay(10);
	Serial.println();
	Serial.print("Connexion au Wifi : ");
	Serial.println(ssid);

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("connecté!");
	Serial.println("adresse IP: ");
	Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
	char *str = (char*)malloc(length+1);
	memcpy(str, payload, length);
	str[length] = 0;

	if( strcmp(topic, "focus") == 0 )
		callback_helper_focus(str);
	else
		callback_helper_cmd(str);

	free(str);
}

void reconnect()
{
	while( ! client.connected() )
	{
		Serial.print("Connxion MQTT...");
		//String clientId = "ESP32Client-";
		//clientId += String(random(0xffff), HEX);
		char clientId[50];
		sprintf(clientId, "ESP32Client-%d-%d", current_state.gid, current_state.id);

		if (client.connect(clientId))
		{
			Serial.println("Connecté");
			client.subscribe(topic_all);
			client.subscribe(topic_group);
			client.subscribe(topic_self);
			client.subscribe(topic_focus);
		}
		else
		{
			Serial.print("failed, rc=");
			Serial.print(client.state());
			Serial.println(" try again in 5 seconds");
			delay(5000);
		}
	}
}