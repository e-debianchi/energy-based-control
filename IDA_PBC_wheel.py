import autograd.numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from mylib import MyFormatter
from autograd import grad
from numpy import seterr
seterr('raise')         # needed to spot underflow errors
import warnings
warnings.filterwarnings("ignore", message="Output seems independent of input.")

# system's parameters
m = 10.2
l = 0.1
J1 = m*l**2
J2 = 0.2
g = 9.81

pi = np.pi

# controller's parameters, change them as needed
a1 = 2
a2 = -3
a3 = 5

def H(phi1, phi2, pi1, pi2):
    kin = 0.5*pi1**2/J1 + 0.5*pi2**2/J2
    pot = m*g*l * np.cos(phi1)
    return pot + kin

dHdphi1 = grad(H, 0)
dHdphi2 = grad(H, 1)
dHdpi1 = grad(H, 2)
dHdpi2 = grad(H, 3)

def Vd(phi1, phi2, kp):
    bigphi = (kp/2)*(phi2 - (J1*(a2 + a3))/(J2*(a1 + a2))*phi1)**2
    return bigphi + (J1*g*l*np.cos(phi1)*m)/(a1 + a2)

dVddphi1 = grad(Vd, 0)
dVddphi2 = grad(Vd, 1)

def Hd(phi1, phi2, pi1, pi2, kp):
    kin = (a3*pi1**2 - 2*a2*pi1*pi2 + a1*pi2**2)/(2*(- a2**2 + a1*a3))
    return kin + Vd(phi1, phi2, kp)

dHddpi1 = grad(Hd, 2)
dHddpi2 = grad(Hd, 3)

g1 = (a2*m*g*l)/(a1+a2)
g2 = -(J1*(a2 + a3))/(J2*(a1 + a2))

def tau(phi1, phi2, pi1, pi2, kp, kd):
    
    y = - dHddpi1(phi1, phi2, pi1, pi2, kp) + dHddpi2(phi1, phi2, pi1, pi2, kp)

    tau_es = 0.5*m*g*l*np.sin(phi1) + (a1-a2)/(2*J1)*dVddphi1(phi1, phi2, kp) + (a2-a3)/(2*J2)*dVddphi2(phi1, phi2, kp)
    tau_di = -kd*y

    return tau_es + tau_di

def wheel(t, y, tau, kp, kd):
    phi1, phi2, pi1, pi2 = y
    u = tau(phi1, phi2, pi1, pi2, kp, kd)

    dphi1 = dHdpi1(phi1, phi2, pi1, pi2)
    dphi2 = dHdpi2(phi1, phi2, pi1, pi2)
    dpi1 = -dHdphi1(phi1, phi2, pi1, pi2) - u
    dpi2 = -dHdphi2(phi1, phi2, pi1, pi2) + u

    return [dphi1, dphi2, dpi1, dpi2]

# Initial conditions, default are random values, change them as needed
y0 = np.random.normal(0.0, 1.0, (4,))
# y0 = [0, 0, 0, 0]
# y0 = [pi, pi, 0, 0]

kp = 1
kd = 15
sol = solve_ivp(wheel, (0, 10), y0, method='LSODA', max_step=1e-2, args=(tau, kp, kd))
phi1, phi2, pi1, pi2 = sol.y[0, :], sol.y[1, :], sol.y[2, :], sol.y[3, :]
t = sol.t

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Plot the functions
ax1.plot(t, phi1, label=r'$\psi_1$', color='b')
ax1.plot(t, phi2, label=r'$\psi_2$', color='r')
ax2.plot(t, pi1, label=r'$\pi_1$', color='b')
ax2.plot(t, pi2, label=r'$\pi_2$', color='r')

ax1.yaxis.set_major_formatter(MyFormatter('%g$\pi$'))
base = 2*pi if np.max(np.abs(phi1)) > np.pi or np.max(np.abs(phi2)) > np.pi else pi/2
ax1.yaxis.set_major_locator(MultipleLocator(base=base))

# Set the titles and labels
ax1.set_ylabel('ang. positions [rad]')
ax2.set_ylabel("ang. velocities [rad/s]")
ax2.set_xlabel('time [s]')

ax1.grid()
ax2.grid()
ax1.legend(loc='best')
ax2.legend(loc='best')

# Show the plot
plt.show()