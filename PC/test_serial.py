from commands import *

class TestSerial():
    def readline(self)-> str:
        next_r:str = ""+self.status["next response"]
        self.status["next response"]  = ""
        return next_r.encode()
    
    def write(self, command:str):
        if command == START_COMMAND:
            self.status["posizione"]= self.status["posizione"]+1000
            self.status["next response"]= START_END

    def write(self, command:str):
        if command == ALLIGNE_COMMAND:
            self.status["posizione"]= self.status["posizione"]+2000
            self.status["next response"]= ALLIGNE_END

    def reset_input_buffer(self):
        print("Reset ;-) ")
    

    def __init__(self):
        self.status = {
            "posizione": 0,
            "next response":""
        }


        