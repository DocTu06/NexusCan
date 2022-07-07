import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import matplotlib.ticker as ticker


def isfloat(input):
    try:
        float(input)
        return True
    except:
        return False
def isgoog(data):
    try:

        input = data.decode()
        return True
    except:
        return False
comgood = 0
file = open(r'C:\Users\dochi\Desktop\LoRa.txt', "a")
ports = list(serial.tools.list_ports.comports())
for p in ports:
    if 'Silicon Labs' in p.description:
        print('The receiver is probably ' + str(p))
while comgood == 0:
    try:
        port = input('Type the COM port: ')
        COMport = 'COM' + str(port)
        ser = serial.Serial(COMport, 9600)
        comgood = 1
    except:
        print("Wrong port. Insert another port.")

temp = []
hum = []
prs = []
ax = 0
ay = 0
az = 0
emt = []
air = []
ser.close()
ser.open()
data = ser.readline()
while isgoog(data) == False:
    print(isgoog(data))
    data = ser.readline()
input = data.decode()
numbers = input.split(',')
if len(numbers) == 4:
    if isfloat(numbers[0]):
        temp = [float(numbers[0])]
    if isfloat(numbers[1]):
        prs = [float(numbers[1])]
    if isfloat(numbers[2]):
        hum = [float(numbers[2])]
    if isfloat(numbers[3]):
        air = [float(numbers[3])]
plt.ion()
figure, axis = plt.subplots(2, 2)
while 1:
    data = ser.readline()
    input = data.decode()
    print(input)
    numbers = input.split(',')
    if len(numbers) == 4:
        if isfloat(numbers[0]):
            temp.append(float(numbers[0]))
        if isfloat(numbers[1]):
            if float(numbers[1]) > 0.0:
                prs.append(float(numbers[1]))
        if isfloat(numbers[2]):
            hum.append(float(numbers[2]))
        if isfloat(numbers[3]):
            air.append(float(numbers[3]))
        axis[0,0].plot(temp,color = 'red')
        axis[0,0].set_title("Temperatură")
        axis[0,0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        axis[1,0].plot(prs,color = 'orange')
        axis[1,0].set_title("Presiune")
        axis[1,0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        axis[0,1].plot(hum,color = 'blue')
        axis[0,1].set_title("Umiditate")
        axis[0,1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        axis[1,1].plot(air,color = 'green')
        axis[1,1].set_title("calitatea aerului")
        axis[1,1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.pause(0.1)
        axis[0,0].cla()
        axis[0,1].cla()
        axis[1,0].cla()
        axis[1,1].cla()