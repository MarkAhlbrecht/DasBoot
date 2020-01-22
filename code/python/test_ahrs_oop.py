
import EasyAHRS

ahrs = EasyAHRS.EasyAHRS(senseHatNum=2)
ahrs.warmup()
ahrs.align(alignSamples=100)