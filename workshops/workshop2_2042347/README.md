# Camera application


run ffmpeg over network with compression
```powershell
ffmpeg -f dshow -i video="Microsoft Camera Front" -s 640X360 -vcodec libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -r 15 -g 30 -f rtsp -rtsp_transport udp rtsp://localhost:8554/mystream
```

run ffmpeg via http
```powershell
ffmpeg -f dshow -i video="Microsoft Camera Front" -vcodec mjpeg -q:v 5 -r 15 -s 640x360 -f mjpeg http://localhost:8080/video.mjpg
```

