import wave, struct, sys
waveFile = wave.open(sys.argv[1],'r')
length = waveFile.getnframes()
arrWave = []
output = ""

for i in range(0,length):
  waveData = waveFile.readframes(1)
  data = struct.unpack("<h", waveData)
  arrWave.append((int(data[0])+32768) / 1311)
  #print i
  
for i in xrange(0,len(arrWave),12):
  #print arrWave[i]
  output = output + str(arrWave[i]) + ", "
  
print output[:len(output)-2]