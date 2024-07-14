import numpy as np

class DataController:
    def __init__(self):
        self.data = []
        self.position = None
        self.velocity = None
        self.rotation = None
        self.timestamp = 0
        self.ready = False
        
        self.calibration_steps = 100
        self.current_step = 0
        
        self.acc_bias = None
        self.rot_bias = None
        self.v_decay = 0.982
        self.s_scale = -5
        self.s_decay = 0.995
        
        self.reset()
        
    def reset(self):
        self.current_step = 0
        self.data = []
        self.position = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.velocity = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.rotation = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.acc_bias = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.rot_bias = {
            'alpha': 0,
            'beta': 0,
            'gamma': 0
        }
        self.timestamp = 0
        self.ready = True
    
    def update(self, data):
        if not self.ready:
            raise Exception('DataController is not ready')
            return
        
        self.current_step += 1
        if self.current_step <= self.calibration_steps:
            self.calibrate(data)
            return
        
        self.data.append(data)
        interval = data['interval']
        self.timestamp += interval
        cur_acc = data['acceleration']
        for key in cur_acc:
            cur_acc[key] -= self.acc_bias[key]
            if abs(cur_acc[key]) < 0.05:
                cur_acc[key] = 0
        
        # print('Current acceleration:', cur_acc, interval)
        
        self.velocity['x'] *= self.v_decay
        self.velocity['y'] *= self.v_decay
        self.velocity['z'] *= self.v_decay
        
        # calculate velocity
        self.velocity['x'] += cur_acc['x'] * interval
        self.velocity['y'] += cur_acc['y'] * interval
        self.velocity['z'] += cur_acc['z'] * interval
        
        # decay
        self.position['x'] *= self.s_decay
        self.position['y'] *= self.s_decay
        self.position['z'] *= self.s_decay
        
        # calculate position
        self.position['x'] += self.velocity['x'] * interval * self.s_scale
        self.position['y'] += self.velocity['y'] * interval * self.s_scale
        self.position['z'] += self.velocity['z'] * interval * self.s_scale
        
        # calculate rotation
        cur_rot = data['rotationRate']
        for key in cur_rot:
            cur_rot[key] -= self.rot_bias[key]
        
        self.rotation['x'] += cur_rot['alpha'] * interval
        self.rotation['y'] += cur_rot['beta'] * interval
        self.rotation['z'] += cur_rot['gamma'] * interval
                
    def print_current(self):
        print("Current:", self.position, self.velocity, self.rotation, self.timestamp)
        
    def get_current(self):
        return np.array([self.position['x'], self.position['y'], self.position['z'] + 5,
                         self.rotation['x'], self.rotation['y'], self.rotation['z']  
                         ])
        
    
    def calibrate(self, data):
        cur_acc = data['acceleration']
        cur_rot = data['rotationRate']
        
        for key in cur_acc:
            self.acc_bias[key] += cur_acc[key]
        for key in cur_rot:
            self.rot_bias[key] += cur_rot[key]
            
        if self.current_step == 1:
            print('Calibrating...')
            
        if self.current_step == self.calibration_steps:
            for key in self.acc_bias:
                self.acc_bias[key] /= self.calibration_steps
            for key in self.rot_bias:
                self.rot_bias[key] /= self.calibration_steps
            print('Calibration done:', self.acc_bias, self.rot_bias)
