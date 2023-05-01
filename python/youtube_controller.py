import pyautogui
import serial
import argparse
import time
import logging

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'L','B': 'K','C': 'J','D': 'H'} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        print("update")
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))
            print("lendo")

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))

        # ONE BUTTON: 1 - on, 0 -off
        if data == b'1':
            print("datab1")
            logging.info("KEYDOWN A")
            pyautogui.keyDown(self.mapping.button['A'])
        elif data == b'0':
            print("datab0")
            logging.info("KEYUP A")
            pyautogui.keyUp(self.mapping.button['A'])

        # TWO BUTTON:  3 - on, 2 -off
        elif data == b'3':
            print("datab3")
            logging.info("KEYDOWN B")
            pyautogui.keyDown(self.mapping.button['B'])
        elif data == b'2':
            print("datab2")
            logging.info("KEYUP B")
            pyautogui.keyUp(self.mapping.button['B'])

         # THREE BUTTON:  5 - on, 4 -off
        elif data == b'5':
            print("datab5")
            logging.info("KEYDOWN C")
            pyautogui.keyDown(self.mapping.button['C'])
        elif data == b'4':
            print("datab4")
            logging.info("KEYUP C")
            pyautogui.keyUp(self.mapping.button['C'])

         # FOUR BUTTON:  7 - on, 6 -off
        elif data == b'7':
            print("datab7")
            logging.info("KEYDOWN D")
            pyautogui.keyDown(self.mapping.button['D'])
        elif data == b'6':
            print("datab6")
            logging.info("KEYUP D")
            pyautogui.keyUp(self.mapping.button['D'])

        self.incoming = self.ser.read()

class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        pyautogui.keyDown(self.mapping.button['B'])
        pyautogui.keyDown(self.mapping.button['C'])
        pyautogui.keyDown(self.mapping.button['D'])

        time.sleep(0.1)

        pyautogui.keyUp(self.mapping.button['A'])
        pyautogui.keyUp(self.mapping.button['B'])
        pyautogui.keyUp(self.mapping.button['C'])
        pyautogui.keyUp(self.mapping.button['D'])

        logging.info("[Dummy] Pressed A button")
        logging.info("[Dummy] Pressed B button")
        logging.info("[Dummy] Pressed C button")
        logging.info("[Dummy] Pressed D button")

        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=115200)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
