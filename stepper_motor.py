from time import sleep
import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)

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
        
        def __init__(self, GPIO_pins:list) -> None:
                for pin in GPIO_pins:
                        gpio.setup(pin, gpio.OUT)
                        gpio.output(pin, 0)
                self.pins = GPIO_pins
                self.movement_direction = None
                self.movement_timeout = 0.006
                
        

        def reset_pins(self) -> None:
                for pin in self.pins:
                        gpio.output(pin, 0)
                
        def positive_movement(self) -> None:
                while self.movement_direction == True:
                        for step in self.step_seq:
                                for pin_state, pin in zip(step, self.pins):
                                        gpio.output(pin, pin_state)
                                sleep(self.movement_timeout)
                self.reset_pins()

        def negative_movement(self) -> None:
                while self.movement_direction == False:
                        for step in self.step_seq[::-1]:
                                for pin_state, pin in zip(step, self.pins):
                                        gpio.output(pin, pin_state)
                                sleep(self.movement_timeout)
                self.reset_pins()
        def movement(self) -> None:
                print("movement started!")
                while self.movement_direction != -1:
                    if self.movement_direction == True:
                        self.positive_movement()
                    elif self.movement_direction == False:
                        self.negative_movement()
                    else :
                        sleep(0)
                        self.reset_pins()
                        continue
                    
                    
                self.reset_pins()
