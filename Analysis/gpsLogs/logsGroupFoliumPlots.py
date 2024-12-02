import time

import json as js
import numpy as np
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
import folium
import math
import base64

latMid = 44.3824419

m = folium.Map(location=[latMid, 26.1131572], zoom_start=12.58)
m_per_deg_lat = 111132.954 - 559.822 * math.cos( 2 * latMid ) + 1.175 * math.cos( 4 * latMid);
m_per_deg_lon = 111132.954 * math.cos ( latMid );


dX = 100
dY = 80
dAngleLat = dY/m_per_deg_lat
dAngleLon = dX/m_per_deg_lon

rootDir = './'
dirs = ['G0000000']

n = 0


loc = []
time = []
spectra = []

uSvMaxIdx = 0
uSvMax = 0

for dir in dirs:
    dir = rootDir + dir
    files = [f for f in listdir(dir) if isfile(join(dir, f))]
    print("reading directory " + dir)
    for f in files:
        with open(join(dir, f)) as currentFile:
            lines = [line.rstrip() for line in currentFile]
            for l in lines:
                data = js.loads(l)
                #print(str(data['timestamp']['hour'] + 3) + ' ' + str(data['timestamp']['minute']) + ' ' )
                if (data['location']['fix'] == 1):
                    currentLat = int(data['location']['lat']/dAngleLat)*dAngleLat
                    currentLon = int(data['location']['lon']/dAngleLon)*dAngleLon
                    d = math.sqrt(math.pow(currentLat - 44.4312629, 2) + math.pow(currentLon - 26.0416965, 2))
                    d = math.sqrt(math.pow(currentLat - 44.3505222, 2) + math.pow(currentLon - 26.0492334, 2))
                    t = data['spectrum']['time']
                    if t == 0:
                        t = 1
                    if (t == 1):
                        if (currentLat, currentLon) in loc:
                            # append to element
                            idx = loc.index((currentLat, currentLon))
                            #print('appending to loc[' + str(idx) + '], ' + str(currentLat) + ', ' + str(currentLon))
                            time[idx] += t
                            spectra[idx] = [sum(x) for x in zip(spectra[idx], data['spectrum']['hist'])]
                        else:
                            # new element
                            loc += [(currentLat, currentLon)]
                            time += [t]
                            spectra += [data['spectrum']['hist']]
                n += 1


uSv = []
counts = []
for n in range(len(loc)):
    currentDose = 0
    currentCounts = 0
    for i in range(1023):
        keV = 2.7676*i - 201.57
        if (keV < 0):
            keV = 0
        currentDose += spectra[n][i] * keV
        currentCounts += spectra[n][i]
    currentDose = currentDose * 1.6021773e-16 * 1e6 / (4.51 * 3.0 * 1e-3)
    currentCounts += spectra[n][1023]
    uSv += [currentDose]
    counts += [currentCounts]


for i in range(len(uSv)):
    uSv[i] = uSv[i]*3600.0 / time[i]
    if uSv[i] > uSv[uSvMaxIdx]:
        uSvMaxIdx = i
        uSvMax = uSv[uSvMaxIdx]


uSvScaled = []
for i in range(len(uSv)):
    uSvScaled += [uSv[i]/uSvMax]


print('total entries = ' + str(n))
print('total points = ' + str(len(loc)))
print('max dose rate = ' + str(uSv[uSvMaxIdx]))


width = 3
height = 2
resolution = 300

bins_array = np.arange(1024)

n = len(loc)

for i in range(n):
    
    fig, ax = plt.subplots(1,1)
    histo_array = np.asarray(spectra[i])
    ax.cla()
    graph = ax.plot(bins_array, histo_array)[0]
    fig.gca().relim()
    fig.gca().autoscale_view()
    plt.figtext(0.7, 0.8, 'Counts: ' + str(counts[i]))
    plt.figtext(0.7, 0.7, 'Seconds: ' + str(time[i]))
    
    png = 'foliumPlots/' + str(i) + '.png'
    fig.savefig(png)
    html = '<img src="foliumPlots/' + str(i) + '.png">'

    folium.Rectangle(
        bounds=[[loc[i][0], loc[i][1]], [loc[i][0] + dAngleLat, loc[i][1] + dAngleLon]],
        color="black",
        weight=0.5,
        opacity=1,
        fill=True,
        fill_color="red",
        fill_opacity=uSvScaled[i],
        tooltip="{0:.2f} uSv/h".format(uSv[i]),
        #popup=html(encoded.decode('UTF-8'))
        popup=html
    ).add_to(m)
    
    plt.close()
m.save('foliumMapPlots.html')
