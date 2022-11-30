import math



def getAngleError(x ,y):
    x_error = x
    y_error = y
    dist = math.sqrt(x_error ** 2 + y_error ** 2)
    norm_x = x_error / dist
    norm_y = y_error / dist
    return math.atan2(norm_y, norm_x)

print(getAngleError(1,1)*57.2958)
