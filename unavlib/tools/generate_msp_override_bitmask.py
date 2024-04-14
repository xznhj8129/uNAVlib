from argparse import ArgumentParser

if __name__ == '__main__':
    parser = ArgumentParser(description='Get bitmask integer for msp_set_override command')
    parser.add_argument('channels', metavar='channel', type=int, nargs='+', help='The channel(s) you want to override')
    channels = parser.parse_args().channels
    result = 0
    for position in channels:
        result |= (1 << position-1)
    print("Enter the following into CLI:")
    print("set msp_override_channels = ", result)