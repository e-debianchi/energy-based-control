import autograd.numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from mylib import MyFormatter
from autograd import grad
from numpy import seterr
seterr('raise')         # needed to spot underflow errors

# system's parameters
m1 = 1
m2 = 0.75
l1 = 0.5
l2 = 0.25
g = 9.81

pi = np.pi

# controller's parameters, change them as needed
phi1d = 0
phi2d = 0

def H(phi1, phi2, pi1, pi2):
    term = m1 + m2 - m2*np.cos(phi1 - phi2)**2
    a1 = 1/(l1**2*term)
    ac = -np.cos(phi1 - phi2)/(l1*l2*term)
    a2 = (m1 + m2)/(l2**2*m2*term)

    kin = 0.5*pi1**2*a1 + 0.5*pi2**2*a2 + pi1*pi2*ac
    pot = (m1+m2)*g*l1 * np.sin(phi1) + m2*g*l2 * np.sin(phi2)
    return pot + kin

dHdphi1 = grad(H, 0)
dHdphi2 = grad(H, 1)
dHdpi1 = grad(H, 2)
dHdpi2 = grad(H, 3)

def Ha(phi1, phi2, kp):
    pot = (m1+m2)*g*l1 * np.sin(phi1) + m2*g*l2 * np.sin(phi2)
    return -pot + 0.5*kp * (phi1 - phi1d)**2 + 0.5*kp * (phi2 - phi2d)**2

dHadphi1 = grad(Ha, 0)
dHadphi2 = grad(Ha, 1)

def Hcl(phi1, phi2, pi1, pi2, kp):
  return H(phi1, phi2, pi1, pi2) + Ha(phi1, phi2, kp)

dHcldpi1 = grad(Hcl, 2)
dHcldpi2 = grad(Hcl, 3)

def tau(phi1, phi2, pi1, pi2, kp, kd):
    y1 = dHcldpi1(phi1, phi2, pi1, pi2, kp)
    y2 = -y1 + dHcldpi2(phi1, phi2, pi1, pi2, kp)

    tau2_es = - dHadphi2(phi1, phi2, kp)
    tau1_es = tau2_es - dHadphi1(phi1, phi2, kp)

    tau1_di = - kd*y1
    tau2_di = - kd*y2

    return [tau1_es+tau1_di, tau2_es+tau2_di]

def manipulator(t, y, tau, kp, kd):
    phi1, phi2, pi1, pi2 = y
    tau1, tau2 = tau(phi1, phi2, pi1, pi2, kp, kd)

    dphi1 = dHdpi1(phi1, phi2, pi1, pi2)
    dphi2 = dHdpi2(phi1, phi2, pi1, pi2)
    dpi1 = -dHdphi1(phi1, phi2, pi1, pi2) + tau1 - tau2
    dpi2 = -dHdphi2(phi1, phi2, pi1, pi2) + tau2

    return [dphi1, dphi2, dpi1, dpi2]

# Initial conditions, default are random values, change them as needed
y0 = np.random.normal(0.0, 1.2, (4,))
# y0 = [pi/2, pi/2, 0, 0]
t_span = (0, 10)

# Controller's gains, change them to see the effect
kp = 5
kd = 1

# Solve the system using the solve_ivp function
sol = solve_ivp(manipulator, t_span, y0, method='LSODA', max_step=1e-2, args=(tau, kp, kd))
phi1, phi2, pi1, pi2 = sol.y[0, :], sol.y[1, :], sol.y[2, :], sol.y[3, :]
t = sol.t

# Plot the functions
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

ax1.plot(t, phi1, label=r'$\varphi_1$', color='b')
ax1.plot(t, phi2, label=r'$\varphi_2$', color='r')

ax2.plot(t, pi1, label=r'$\pi_1$', color='b')
ax2.plot(t, pi2, label=r'$\pi_2$', color='r')

ax1.yaxis.set_major_formatter(MyFormatter('%g$\pi$'))
ax1.yaxis.set_major_locator(MultipleLocator(base=np.pi/4))

# Set the titles and labels
ax1.set_ylabel('ang. positions [rad]')
ax2.set_ylabel("ang. momentum [$m^2\,kg$/s]")
ax2.set_xlabel('time [s]')

ax1.grid()
ax2.grid()
ax1.legend(loc='best')
ax2.legend(loc='best')

# Show the plot
plt.show()