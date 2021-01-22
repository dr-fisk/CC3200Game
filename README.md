# CC3200Game
A small game which uses the CC 3200 by Texas Instruments.
The game uses the gyroscope present on the table along with an OLED screen which is cotrolled using i2c to draw the graphics on screen.
There is further integration with AWS to send a text message after a game over.
The game contains a collision detection system for when the player comes in contact with a falling object.
Two switches can speed up the game or slow down the game.
To move the player, simply move the cc3200 and the gyroscope will send its values and be interpreted by the software.

# How to use
Have CCS installed on your computer. Open the IDE and import the project. Press build on the project. Ensure that the CC3200 is plugged into a usb port on your computer.
Then flash the project using uniflash by TI. The game should then automatically run on your board.
