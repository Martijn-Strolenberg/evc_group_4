from enum import IntEnum
from typing import Callable
import Jetson.GPIO as GPIO
import rospy
# import threading

from ledClass import ButtonLED


class ButtonEvent(IntEnum):
    PRESS = 0
    RELEASE = 1


class ButtonDriver:
    """Class handling communication with a GPIO button.

    Args:
        led_gpio_pin (:obj:`int`): ID of the pin the button LED is connected to.
        signal_gpio_pin (:obj:`int`): ID of the pin the button (signal) is connected to.
        callback (:obj:`callable`): callback function to receive signal events.
    """

    def __init__(self, led_gpio_pin, signal_gpio_pin, callback):
        # valid gpio pin
        if not 1 <= signal_gpio_pin <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
        # validate callback
        if not callable(callback):
            raise ValueError("The callback object must be a callable object")
        self._callback = callback
        # configure GPIO pin
        self._signal_gpio_pin = signal_gpio_pin
        GPIO.setmode(GPIO.BOARD)
        #GPIO.setup(self._signal_gpio_pin, GPIO.IN)
        GPIO.setup(self._signal_gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # create LED object
        self._led = ButtonLED(led_gpio_pin)
        # prevent_default when performing tests
        self._prevent_default = False
        # callback when test finishes, only supplied when starting a test
        self._test_callback = None
        # attach event listeners to the signal GPIO pin
        #GPIO.add_event_detect(self._signal_gpio_pin, GPIO.BOTH, callback=self._cb)
        GPIO.add_event_detect(self._signal_gpio_pin, GPIO.BOTH, callback=self._cb, bouncetime=200)


    @property
    def led(self):
        return self._led

    def _cb(self, pin):
        signal = int(GPIO.input(pin))
        event = ButtonEvent(signal)
        rospy.loginfo("GPIO callback triggered, pin: %d, signal: %d", pin, signal)
        # when running tests
        if self._prevent_default:
            if event == ButtonEvent.RELEASE:
                if isinstance(self._test_callback, Callable):
                    self._test_callback()
                self.finish_test()
            return

        self._callback(event)
        # # Run the callback in a new thread
        # threading.Thread(target=self._callback, args=(event,)).start()

    def start_test(self, test_cb):
        self._test_callback = test_cb
        self._prevent_default = True

    def finish_test(self):
        self._prevent_default = False
        self._test_callback = None

    def shutdown(self):
        # remove event listeners
        GPIO.remove_event_detect(self._signal_gpio_pin)
        # shutdown LED controller
        if hasattr(self, "_led"):
            self._led.shutdown()
