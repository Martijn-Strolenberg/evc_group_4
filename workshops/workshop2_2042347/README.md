# Laptop Camera application

run the server
```powershell
.\mediamtx.exe
```

run ffmpeg over network with compression
```powershell
ffmpeg -f dshow -i video="Microsoft Camera Front" -s 640X360 -vcodec libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -r 15 -g 30 -f rtsp -rtsp_transport udp rtsp://localhost:8554/mystream
```

build an run the docker container:
```powershell
docker build -t evc_ros:v1 .
```
```powershell
docker run --rm -it -e DISPLAY=$DISPLAY --name jetson_home -p 11311:11311 -p 45100-45101:45100-45101 evc_ros:v1
```
