I want to make a simple multiplayer web minigame meant to be played primarily on mobile phone.

It is an arcady style 3D game where you need to push other players off the arena. The players are balls
that are controlled by the different users. Before getting in the game, the player get into a lobby and connect to a host 
that will be responsible for the game state. The host will be one of the players, and the game is peer to peer with WebRTC,
with the connection being established through PeerJS. The host will be responsible for synchronizing the game state between the different players, and for handling the game logic.

When getting in the game, the user can choose to either play against bots, host a game or join a game. If the user chooses to host a game, they will be given a code that they can share with their friends to join the game. If the user chooses to join a game, they will be prompted to enter the code of the game they want to join.

I want this game to be very simple, but immersive where you feel like you play with the players around you. So I want this to be an *Augmented Reality* experience, where the player points the camera to the center of a table, where there will be a marker, on top of which the arena will be. The players will gather around the marker and each have their own point of view around.

The players can disable the AR experience and play without a camera (but still with the camera moving around), where the arena is just on the screen, but I want the AR experience to be the main focus.

There is an on screen joystick on the left, and 2 buttons on the right, jump (which makes the ball jump if on the ground) and boost (which gives a quick dash impulse in the direction of the joystick, on a cooldown of 1s)

This game should be a static page deployed on github pages, without any backeend. It should feel responsive and smooth, with a simple and colorful art style. There should be client side prediction for smooth gameplay, with lag compensation and server reconciliation.

The game should be easy to pick up and play. It should be playable on both iOS and Android devices, and should not require any additional software or plugins to be installed, other than chrome.

Use vite as the build tool, strict typescript, and use a Three.js for rendering the game. For the AR experience, use a library like AR.js if necessary to handle the marker tracking and rendering of the arena in the real world. I expect the tracker to be a QR code which is the website where the game is hosted (gh pages url). The tracking should be smooth and responsive, with low latency, to ensure a good user experience. The game should also be optimized for mobile devices, with touch controls and a responsive UI.
