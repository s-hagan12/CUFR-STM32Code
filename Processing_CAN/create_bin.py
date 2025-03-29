import sys
import pandas as pd

#To run this, use "python create_bin.py <path to csv>" 


def main(args):
    if len(args) != 1:
        print("No path provided.")
        return

    csv_file = None
    try:
        csv_file = pd.read_csv(args[0])
    except:
        print("failed to open file")     
        return
    
    f = open("settings.bin", "wb")
    #Number of messages
    msg_count = int(csv_file.iat[0, 0])
    t=msg_count.to_bytes()
    f.write(t)

    for i in range(3):
        id = csv_file.iat[i+1, 0].encode('utf-8')
        length = int(csv_file.iat[i+1, 1]).to_bytes()
        speed = int(csv_file.iat[i+1, 2]/10).to_bytes()
        num_messages = int(csv_file.iat[i+1, 3]).to_bytes()
        f.write(id)
        f.write(length)
        f.write(speed)
        f.write(num_messages)

    med_idx = 3
    med_i = 0
    fast_idx = 3
    fast_i = 0
    stat_idx = 3
    stat_i = 0
    for i in range(msg_count):
        msg_id = int(csv_file.iat[i+4, 0], 16).to_bytes(2)
        f.write(msg_id)
        pkt = csv_file.iat[i+4, 4]
        f.write(pkt.encode('utf-8'))
        start = 0
        idx = 0
        len_to_write = int(int(csv_file.iat[i+4, 3])/8) 
        if pkt == "s":
            start = stat_idx
            stat_idx+=len_to_write
            idx = stat_i
            stat_i += 1
        elif pkt == "m":
            start = med_idx
            med_idx+=len_to_write
            idx = med_i
            med_i += 1
        elif pkt == "f":
            start = fast_idx
            fast_idx+=len_to_write
            idx = fast_i
            fast_i += 1
        f.write(start.to_bytes())
        data_len = int(int(csv_file.iat[i+4, 1])/8)
        f.write(data_len.to_bytes())
        bytes_to_send = str("0xFFFFFF")
        # if data_len != len_to_write:
        #     bytes_to_send = csv_file.iat[i+4, 2]
        b_t_s = int(bytes_to_send, 16)
        f.write(b_t_s.to_bytes(3))
        f.write(len_to_write.to_bytes())
        f.write(idx.to_bytes())


        
    
    #print(csv_file)

    f.close()

if __name__ == "__main__":
    main(sys.argv[1:])





