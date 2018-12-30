import paho.mqtt.client as mqtt
#import paho.mqtt.subscribe as subscribe
import RPi.GPIO as GPIO
from signal import pause
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image

# Raspberry Pi pin configuration:
RST = 24
# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

mqttDeviceID = "medTracker_pi_zero_w"
mqttServer = "192.168.1.10"
drugTimeTopic = "medtracker/drugtime"
medsNotTakenTopic = "medtracker/medsnottaken"
mqttClient = mqtt.Client("medTracker_pi_zero_w")

GPIO.setmode(GPIO.BCM) 

GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)#leftButton
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)#rightButton

imagePills = Image.open('/home/pi/medTrackerButton/icons/pill_small_icon.pbm').convert('1')
imageAlarm = Image.open('/home/pi/medTrackerButton/icons/alarm_icon.pbm').convert('1')
imageAlert = Image.open('/home/pi/medTrackerButton/icons/alert_icon.pbm').convert('1')

def leftButtonPress(channel):
    print("leftButtonPress! do nothing")
    start_time = time.time()

    while GPIO.input(channel) == 0: # Wait for the button up
        pass
    buttonTime = time.time() - start_time
    if 0 <= buttonTime < 2:        # Ignore noise
        print("short press")        # 1= brief push

    elif 2 <= buttonTime < 4:         
        print("medium press")        # 2= Long push

    elif buttonTime >= 4:
        print("long press") 

def rightButtonPress(channel):
    print("rightButtonPress! send event")
    #print(channel)
    mqttClient.publish("medtracker/medstaken", mqttDeviceID)

def clearDisplay():
    disp.clear()
    disp.display()

def displayImage(image):
    disp.image(image)
    disp.display()

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_message(client, userdata, message):
    print("%s %s" % (message.topic, message.payload))

def on_drugTime(client, userdata, message):
    #print("DrugTime: %s %s" % (message.topic, message.payload))
    if(message.payload == "off"):
        clearDisplay()
    else:
        displayImage(imagePills)

def on_medsNotTaken(client, userdata, message):
    #print("MedsNotTaken: %s %s" % (message.topic, message.payload))
    if(message.payload == "1"):
        displayImage(imageAlarm)
    elif(message.payload == "2"):
        displayImage(imageAlert)
    elif(message.payload == "3"):
        displayImage(imageAlert)
    else:
        clearDisplay()

GPIO.add_event_detect(17, GPIO.FALLING, callback=rightButtonPress, bouncetime=300)
GPIO.add_event_detect(4, GPIO.FALLING, callback=leftButtonPress, bouncetime=300)

mqttClient.message_callback_add(drugTimeTopic, on_drugTime)
mqttClient.message_callback_add(medsNotTakenTopic, on_medsNotTaken)
mqttClient.on_subscribe = on_subscribe
mqttClient.on_message = on_message
mqttClient.connect(mqttServer)
mqttClient.subscribe(drugTimeTopic, 2)
mqttClient.subscribe(medsNotTakenTopic, 2)

# Initialize display.
disp.begin()
clearDisplay()
#image = Image.open('icons/skullgimp.bmp').resize((disp.width, disp.height), Image.ANTIALIAS).convert('1')

try:  
    print("Ready")
    #pause()
    mqttClient.loop_forever()
except KeyboardInterrupt:  
    GPIO.cleanup()       # clean up GPIO on CTRL+C exit  
GPIO.cleanup()     