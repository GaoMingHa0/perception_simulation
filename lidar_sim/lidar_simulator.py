import numpy as np



def judge(car_position, point):
    # 四个判断

    

def angle_judge(car_position,car_heading, cone_position):
    # 角度判断
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]

    angle_to_cone = np.arctan2(dy,dx)

    relative_angle = angle_to_cone - car_heading

    relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

    half_fov = 

def point_make_and_map(i):



def distance(car_position, point):
    if()


def main():
    car_position = np.array([0, 0, 0])
    for i in arr:
        if(front(car_position,i)):
            if(judge(car_position,i)):
                point_make_and_map(i)



if "__main__" == __name__:
    main()