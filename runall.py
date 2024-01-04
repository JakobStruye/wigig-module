import os
import time
import subprocess
txSizes = ["Small"]#, "Big"]
rxSizes = ["Big", "Small"]#, "BigAz", "Small"]
beamformings = ["covrage" , "none", "sectors"]
fpss = [100]
dataRates = ["2000Mbps", "5000Mbps", "7000Mbps", "8000Mbps"]
predictions=["device", "model", "oracle"]
motions=["Low", "Mixed", "High"]
bhiConfigs = ["default", "optimized"]
dtibfs = ["true", "false"]
covrageIntervals = [100, 1000]

counter = 0
for txSize in txSizes:
 for rxSize in rxSizes:
  for beamforming in beamformings:
   for fps in fpss:
    for dataRate in dataRates:
     for prediction in predictions:
      for motion in motions:
       for bhiConfig in bhiConfigs:
        for dtibf in dtibfs:
         for covrageInterval in covrageIntervals:
          if rxSize == "Small" and beamforming == "covrage": #doesnt work
           continue
          if rxSize == "Big" and beamforming == "sectors": #doesnt work
           continue
          if prediction != "device" and beamforming != "covrage": #doesnt work
           continue
          if covrageInterval != covrageIntervals[0] and dtibf == "false":
           continue
          counter += 1
          print(counter)
          command = ('./waf --run "evaluate_qd_channel_lroom_scenario --txSize='+txSize +
                     ' --rxSize='+rxSize + ' --beamforming='+beamforming +
                     ' --fps='+str(fps) + ' --dataRate='+dataRate + ' --prediction='+prediction +
                     ' --motion='+motion + ' --bhiConfig='+bhiConfig + ' --dtibf='+dtibf +
                     ' --covrageInterval='+str(covrageInterval)
                     +'"')
          command += "&"
          os.system(command)
          time.sleep(5)
          while True:
            result = subprocess.check_output('ps aux | grep evaluate_qd_channel_lroom_scenario | wc -l ', shell=True, text=True)
            running = (int(result) -2)/2
            if (running < 1):
                break

