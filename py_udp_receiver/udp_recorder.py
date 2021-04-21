import socket
import wave
import os

def udp_init():

    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    # bind to port
    sock.bind(('', 55726))

    return sock


if __name__ == "__main__":

    # output file path management
    fn = 'sample.wav'
    fp = os.path.join(os.getcwd(), fn)

    # variable init
    buffer = bytearray()
    n = 0
    offset = False
    samples = list()
    bit_depth = 16
    byte_depth = 2

    # udp connection
    sock = udp_init()

    while n < 1000:
        data, addr = sock.recvfrom(2048)   # buffer size is 1024 bytes
        n += 1

        # align for offset
        if offset == True:
            sample = int.from_bytes(data, byteorder='little', signed=False) - int(2 ** bit_depth / 2)
            data = sample.to_bytes(byte_depth, byteorder='little', signed=True)

        buffer.extend(data)
        samples.append(int.from_bytes(data, byteorder='little', signed=offset))

    # save as wave file
    wav_obj = wave.open(fp, mode='w')   # create wave write object
    wav_obj.setnchannels(1)             # mono/stereo
    wav_obj.setsampwidth(byte_depth)    # bytes per frame
    wav_obj.setnframes(n)               # total number of samples
    wav_obj.setframerate(256000)        # sample rate
    wav_obj.writeframesraw(buffer)      # pass bytes to wave object
    wav_obj.close()                     # close file

    # plot
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(range(len(samples)), samples)
    plt.waitforbuttonpress()
