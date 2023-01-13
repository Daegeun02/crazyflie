from time import sleep



if __name__ == "__main__":

    t = 0

    while t < 10:

        if t % 2 == 0:
            pass
        else:
            t += 1
            continue

        print('even')

        sleep(1)
        t += 1