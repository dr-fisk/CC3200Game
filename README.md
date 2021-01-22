# CC3200Game
A small game which uses the CC 3200 by Texas Instruments.
The game uses the gyroscope present on the table along with an OLED screen which is cotrolled using i2c to draw the graphics on screen.
There is further integration with AWS to send a text message after a game over.
The game contains a collision detection system for when the player comes in contact with a falling object.
Two switches can speed up the game or slow down the game.
To move the player, simply move the cc3200 and the gyroscope will send its values and be interpreted by the software.
