import binascii


def FinalResultDecode(result):
    HexString = binascii.hexlify(result).decode()
    FinalTorque = ''.join(chr(byte) for byte in result[15:20])
    print('原始数据为: ', HexString)
    print('解析数据为: ', FinalTorque)