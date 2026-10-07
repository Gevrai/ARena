# Research notes — tracker & platform (2026-10-07)

**CV pipeline**
- Write our own in TS. Port algorithms from **js-aruco2** 2.0.0 (MIT, no types, plain JS): `src/cv.js` has `adaptiveThreshold`, `findContours`, `approxPolyDP`, `isContourConvex`, `perimeter`, `warp`, `getPerspectiveTransform`, `square2quad`. The core uses no DOM, so it works in a worker. Repo: https://github.com/damianofalcioni/js-aruco2
- Rejected: opencv.js (about 10 MB), AR.js (heavy, tied to A-Frame/three), jsfeat (dead, no contours).

**Pose from the 4 corners** (no maintained IPPE package on npm)
1. Compute homography H from the corners with a square→quad solve.
2. r1 = λK⁻¹h1, r2 = λK⁻¹h2, r3 = r1×r2, t = λK⁻¹h3.
3. Orthonormalise R (SVD or Gram-Schmidt).
4. Optionally refine with Gauss-Newton on reprojection error.

A small tilt makes the pose ambiguous between two solutions: disambiguate with the QR finder patterns and the gyro.

**Marker practicalities**
- Leave a white quiet zone between the frame and the QR.
- Reject inner quads (finder squares): keep the largest convex quad.
- Watch for glare and blur on glossy cards.
- Minimum size: about 40–50 px wide in a 640 px frame; aim for 60–80 px or more.
- At 640 px a 50 mm card reaches roughly 0.5–0.6 m. For more range, raise the detection resolution.

**Intrinsics**
- The browser does not expose FOV. Assume an HFOV of about 65°, so fx ≈ 0.78 × the video width.
- Use the actual video stream's size and aspect ratio, not the screen's.

**DeviceOrientation → quaternion** (three-stdlib `DeviceOrientationControls`)
1. `euler.set(beta, alpha, -gamma, 'YXZ')`
2. multiply by q1 = (-√0.5, 0, 0, √0.5)
3. multiply by axisAngle(z, -screen.orientation.angle)

On iOS, call `DeviceOrientationEvent.requestPermission()` inside a user gesture. On iOS, alpha is relative.

**iOS Chrome (WebKit)**
- No WebXR and no BarcodeDetector.
- `getUserMedia` works over HTTPS; set `playsinline` and `muted` on the video.
- No element fullscreen on iPhone (iPad has it).
- "Add to Home Screen" from Chrome needs iOS ≥ 16.4.
- Add the `apple-mobile-web-app-capable` meta tag and `viewport-fit=cover`.

**QR codes**
- `uqr` 0.1.3: tiny, typed, outputs an SVG string or a boolean matrix. Works both in the build script and in the browser.

**PeerJS** 1.5.5
- `reliable:false` only makes the channel unordered; it still retransmits.
- For truly lossy delivery, create our own RTCDataChannel (`maxRetransmits: 0`) on the underlying `peerConnection`.
- The public 0.peerjs.com server was up as of 2026-10-07.
