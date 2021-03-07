from os import wait
import time

SERIAL_PORT = '/dev/ttyACM0'
CSV_FILE = './data.csv'

waitdelta = 20

def readLine(serialport, buffer, delimiter='\n'):
    buffer += serialport.readline()
    delim = buffer.find(delimiter)

    if delim == 0:
        return None

    ret = buffer[:delim]
    buffer = buffer[delim + 1:]

    return ret
    

def main():
    now = time.time()
    buffer = ''

    while time.time() < now + waitdelta:

        try:
            serialport = open(SERIAL_PORT, 'r')
            csv = open(CSV_FILE, 'a+')
            time.sleep(3)
            serialport.readline()
            serialport.readline()
            
            while time.time() < now + waitdelta:
                
                line = readLine(serialport, buffer)
                
                if line is not None and len(line) > 0:
                    line = '{},{}\n'.format(int(time.time() * 1000), line)
                    csv.write(line)
                    time.sleep(0.5)
            
        except:
            print('No serial port detected...')
            time.sleep(2)
        
    try:
        serialport.close()
    except:
        pass

    try:
        csv.close()
    except:
        pass

if __name__ == '__main__':
    main()
