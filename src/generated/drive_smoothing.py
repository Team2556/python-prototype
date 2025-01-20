def smooth(joystick_value: float, blend = 0.6):
    '''
    Add curve to joystick to smooth movements'''
    # blend_value = joystick_value * blend
    # smoothed_blend_value = blend_value ** 9
    # blend_mod = (1 - blend) * joystick_value
    # smoothed_value = smoothed_blend_value + blend_mod
    
    # # Limit smoothed vlaue to be between -1 and 1
    # if smoothed_value > 1:
    #     smoothed_value = 1
    # elif smoothed_value < -1:
    #     smoothed_value = -1
    
    # return smoothed_value
    smoothed_value = joystick_value ** 3
    
    
    
    # if smoothed_value < 0.001:
    #     return 0
    # if smoothed_value < 0.01:
    #     return 0.01
    
    return smoothed_value
    
if __name__ == '__main__':
    while True:
        test_input = input('Value to Test: ')
        print(smooth(float(test_input)))
    
    