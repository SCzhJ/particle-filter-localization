import numpy as np
def WrapToPosNegPi(theta):
    if theta > 0:
        theta = theta % (2*np.pi)
        if theta > np.pi:
            theta = theta - 2*np.pi
    elif theta < 0:
        theta = abs(theta) % (2*np.pi)
        if theta > np.pi:
            theta = -(theta - 2*np.pi)
    return theta

if __name__=="__main__":
    a = -11
    print(np.cos(a))
    a_ = WrapToPosNegPi(a)
    print(a_)
    print(np.cos(a_))

    for i in range(100):
        cos_i_pos = round(np.cos(i),8)
        i_w_pos = WrapToPosNegPi(i)
        cos_w_pos = round(np.cos(i_w_pos),8)
        if i_w_pos > np.pi or i_w_pos < -np.pi:
            print("fail pos range")
        if cos_w_pos!=cos_i_pos:
            print("fail pos val")
            print(cos_i_pos,np.cos(i_w_pos))

        cos_i_neg = round(np.cos(-i),8)
        i_w_neg = WrapToPosNegPi(-i)
        cos_w_neg = round(np.cos(i_w_neg),8)
        if i_w_neg > np.pi or i_w_neg < -np.pi:
            print("fail neg range")
        if cos_w_neg!=cos_i_neg:
            print("fail neg val")
            print(cos_i_neg,np.cos(i_w_neg))

