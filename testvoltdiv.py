import sys

sys.path.append(r'/home/rho/Python/pi/havsmolf/')
from utill import VoltDiv


def main():

    vd = VoltDiv(11, 3000, 680, vcircuit=3.3, dscale=1023)
    print("Volt in")
    print("Expect 11: {}".format(vd.evalVin(630)))
    print("Expect 630: {}".format(vd.reverseVin(11)))

    print("Engin ground r2 unknown")
    vd = VoltDiv(3.3, 680, 300, vcircuit=3.3, dscale=1023)
    print("Expect 300: {}".format(vd.evalR2(313)))
    print("Expect 313: {}".format(vd.reverseR2(300)))
    vd = VoltDiv(3.3, 470, 100, vcircuit=3.3, dscale=1023)
    print("Expect 100: {}".format(vd.evalR2(179)))
    print("Expect 179: {}".format(vd.reverseR2(100)))

    print("Return r1 unknown")
    vd = VoltDiv(3.3, 70, 330, vcircuit=3.3, dscale=1023)
    print("Expect 70: {}".format(vd.evalR1(844)))
    print("Expect 844: {}".format(vd.reverseR1(70)))


if __name__ == "__main__":
    main()
