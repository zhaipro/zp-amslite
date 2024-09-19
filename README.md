# zp-ams

初次使用请连接esp32发起的热点WiFi，后访问 `http://192.168.4.1` 完成初步配置：
名称：`zhaipro-amslite`
密码：`zhaipro-amslite`



https://marlinfw.org/docs/gcode/M073.html
M73 P<percent> [R<minutes>]
https://forum.bambulab.com/t/bambu-lab-x1-specific-g-code/666
M400 U1 ; pause for user intervention

M73 P{110+next_extruder} R[next_extruder]
