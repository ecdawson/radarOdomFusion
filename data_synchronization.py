import numpy as np


class data_synch:
    def __init__(self,riss,radar):
        self.riss = riss
        self.radar = radar

    def get_synched_samples(self,index_rad,index_riss):
        
        i_radar = index_rad
        i_riss = index_riss
        radar_next = self.radar[i_radar,0]
        riss_next = self.riss[i_riss,0]
        next_time_diff = self.riss[i_riss,0]-self.radar[i_radar,0]

        if next_time_diff < -5:
                while next_time_diff<-5:
                        i_riss = i_riss + 1
                        next_time_diff = self.riss[i_riss,0]-self.radar[i_radar,0]
            
        if next_time_diff > 5:
                while next_time_diff>5:
                        i_radar = i_radar +1
                        next_time_diff = self.riss[i_riss,0]-self.radar[i_radar,0]


        cur_time = self.radar[i_radar,0]
        return self.radar[i_radar,:],self.riss[i_riss],i_radar,i_riss,cur_time

    def is_gap(self,current_time,next_time):
        time_difference = next_time - current_time
        
        if time_difference > 200:
            gap = 1
        else:
            gap = 0

        return gap,time_difference

    def is_gap_significant(self,time_difference,threshold = 1000):
        
        if time_difference > threshold:
            return 1
        else:
            return 0
        

        
        
