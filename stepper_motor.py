from time import sleep
import RPi.GPIO as gpio

class StepperMotor:
        step_seq = [
                [1, 0, 0, 0],
                [1, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 1, 0],
                [0, 0, 1, 1],
                [0, 0, 0, 1],
                [1, 0, 0, 1],
        ]
        
        def __init__(self, GPIO_pins:list, movement_direction, full_step_timeout) -> None:
                self.pins = GPIO_pins
                self.movement_direction = movement_direction
                self.full_step_timeout = full_step_timeout
                gpio.setmode(gpio.BOARD)
                for pin in GPIO_pins:
                        gpio.setup(pin, gpio.OUT)
                        gpio.output(pin, 0)
                
        

        def reset_pins(self) -> None:
                for pin in self.pins:
                        gpio.output(pin, 0)
                
        def positive_movement(self) -> None:
                while self.movement_direction.value == 1:
                        for step in self.step_seq:
                                for pin_state, pin in zip(step, self.pins):
                                        gpio.output(pin, pin_state)
                                sleep(self.full_step_timeout.value)
                self.reset_pins()

        def negative_movement(self) -> None:
                while self.movement_direction.value == -1:
                        for step in self.step_seq[::-1]:
                                for pin_state, pin in zip(step, self.pins):
                                        gpio.output(pin, pin_state)
                                sleep(self.full_step_timeout.value)
                self.reset_pins()
        def movement(self) -> None:
                print("movement started!")
                while True:
                        if self.movement_direction.value == 1:
                                self.positive_movement()
                        elif self.movement_direction.value == -1:
                                self.negative_movement()
                        else :
                                sleep(0)
                                self.reset_pins()
