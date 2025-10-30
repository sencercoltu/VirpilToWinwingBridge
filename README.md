# Virpil Constellation Alpha Grip To Winwing Orion 2 Base Bridge

Simple bridge firmware to connect a Virpil Constellation Alpha grip to a Winwing Orion 2 base using an STM32F411CEU6 ("Blackpill").

Winwing doesn't sell the z-axis twist module anymore, so I bought a Virpil stick without a base to use it with my Wining Orion 2 Base. With 5to6pin adapter, Virpil stick is recognised as a Thrustmaster stick (SPI). Some buttons are mapped incorrectly, some are not working, and there is no twist axis.
By translating/forwarding the Virpil Constellation Alpha grip messages (SPI) to Winwing Orion 2 base (Half-duplex UART), all buttons and axes of the Virpil stick can now be used.
(Inverse logic may be applied to use Winwing sticks with Virpil bases)

- Replace 5 pin mini DIN cable with 6 pin one. 
- Virpil grip is recognised as a Winwing ViperAce EX stick with shaker and analog stick.
- All Virpil buttons/axes are mapped to the ViperAce stick.
- Axes calibration is started by holding Red + Black + Dial button for 3 seconds. Then move all axes (pinky lever, twist, thumbstick) to min/max. While in calibration, press red button to set pinky detent1, black button for detent 2. Hold dial button to save values and exit calibration mode. Replug usb cable from base.

<a href="https://photos.app.goo.gl/dnXUHSBZXFb26ghDA">Video of mapped buttons/axes</a>
<br>
<hr>
<br>
<img width="300" alt="image" src="https://github.com/user-attachments/assets/fbe10109-d730-4932-8247-ba2d27c5f2cf" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/3ef0bcd5-d66e-4826-8a37-41e1306c72c4" />
<img width="600" alt="image" src="https://github.com/user-attachments/assets/480d532d-78c3-4107-a019-3cf5b2f40599" />
<img width="600" alt="image" src="https://github.com/user-attachments/assets/c2ddc623-c715-448d-9ed8-1199f71086be" />
