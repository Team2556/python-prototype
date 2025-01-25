def smooth(joystick_value: float, blend = 0.1):
    '''
    Add curve to joystick to smooth movements'''
    exponential_for_curving = 3
    #make sure it is odd
    assert (exponential_for_curving-1) % 2 == 0

    blend_value = joystick_value * blend
    smoothed_blend_value = blend_value ** exponential_for_curving
    blend_mod = (1 - blend) * joystick_value
    smoothed_value = smoothed_blend_value + blend_mod
    
    # Limit smoothed vlaue to be between -1 and 1
    if smoothed_value > 1:
        smoothed_value = 1
    elif smoothed_value < -1:
        smoothed_value = -1
    
    # # return smoothed_value
    # smoothed_value = joystick_value ** 3

    
    return smoothed_value
    
if __name__ == '__main__':
    while True:
        test_input = input('Value to Test: ')
        print(smooth(float(test_input)))
    
    