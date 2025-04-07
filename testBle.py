import threading
import signal
import asyncio

import sensors as sen


def bme(chari, data):
    bmeDict = sen.esp32HavsmolfDataBme(data)
    print(bmeDict)


def icm(chari, data):
    icmDict = sen.esp32HavsmolfDataIcm(data)
    print(icmDict)


async def mainAsync(ble):
    async with asyncio.TaskGroup() as tg:
        _ = tg.create_task(ble.runBle(bme, icm))


def run(ble):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(mainAsync(ble))


def main():
    with sen.BleEsp("/dev/ttyACM0",
                    200,
                    3,
                    "34:85:18:7B:AE:E5") as ble:
        work = threading.Thread(
            target=run,
            args=(ble,)
        )
        work.start()
        signals = {signal.SIGINT, signal.SIGTERM}
        s = signal.sigwait(signals)
        print("Server was interumpted signo: {}".format(s))
        ble.stopBle()
        work.join()
        print("asyncio thread done")


if __name__ == "__main__":
    main()
