# IKEA Fyrtur roller blind esphome templates

## Explanation

Here are examples if you want to integrate this with ESPHome, just solder an ESP to the fytur-board and flash esphome on it.

I added one yaml with deepsleep, because without it, the battery only holds for about 3 days, this also has the disadvantage that you have to make scripts for openin/closing that wait until the ESP comes online.
I have used the uptime-sensor to see if the ESP is online. You also have to create a "input_boolean.esphome_deep_sleep" sensor in home assistant, so you can enable/disable deep sleep (for if you want to update and stuff).