from signal import signal, SIGINT
from goprocam import GoProCamera, constants
import sys
#172.27.179.51:8080
gopro = GoProCamera.GoPro(ip_address='172.27.179.51:8080')


def handler(s, f):
    gopro.stopWebcam()
    quit()


signal(SIGINT, handler)

try:
    gopro.setWiredControl(constants.off)
except:
    pass
gopro.startWebcam(constants.Webcam.Resolution.R720p)
gopro.webcamFOV(constants.Webcam.FOV.Linear)
gopro.getWebcamPreview()
gopro.KeepAlive()