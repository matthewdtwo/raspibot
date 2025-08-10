
import qwiic_dual_encoder_reader

class Encoders:
    def __init__(self):
        self.encoders = qwiic_dual_encoder_reader.QwiicDualEncoderReader()

        if self.encoders.connected == False:
            print("Encoders are not connected")
            return

        self.encoders.begin()

        self.encoders.set_count1(0)
        self.encoders.set_count2(0)

    def get_counts(self):
        left_count = self.encoders.count1
        right_count = self.encoders.count2
        return left_count, right_count
    
    def reset_counts(self):
        self.encoders.set_count1(0)
        self.encoders.set_count2(0)

        
