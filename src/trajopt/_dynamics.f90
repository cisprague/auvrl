
subroutine lagrangian(u_t, u_x, u_y, alpha, L)
implicit none
REAL*8, intent(in) :: u_t
REAL*8, intent(in) :: u_x
REAL*8, intent(in) :: u_y
REAL*8, intent(in) :: alpha
REAL*8, intent(out) :: L

L = alpha*u_t + u_t**2*(-alpha + 1)

end subroutine

subroutine hamiltonian(x, y, vx, vy, theta, omega, lambda_x, lambda_y, &
      lambda_v_x, lambda_v_y, lambda_theta, lambda_omega, u_t, &
      u_x, u_y, alpha, g, T, m, l, rho, C_d, A, H)
implicit none
REAL*8, intent(in) :: x
REAL*8, intent(in) :: y
REAL*8, intent(in) :: vx
REAL*8, intent(in) :: vy
REAL*8, intent(in) :: theta
REAL*8, intent(in) :: omega
REAL*8, intent(in) :: lambda_x
REAL*8, intent(in) :: lambda_y
REAL*8, intent(in) :: lambda_v_x
REAL*8, intent(in) :: lambda_v_y
REAL*8, intent(in) :: lambda_theta
REAL*8, intent(in) :: lambda_omega
REAL*8, intent(in) :: u_t
REAL*8, intent(in) :: u_x
REAL*8, intent(in) :: u_y
REAL*8, intent(in) :: alpha
REAL*8, intent(in) :: g
REAL*8, intent(in) :: T
REAL*8, intent(in) :: m
REAL*8, intent(in) :: l
REAL*8, intent(in) :: rho
REAL*8, intent(in) :: C_d
REAL*8, intent(in) :: A
REAL*8, intent(out) :: H

H = 6*T*lambda_omega*u_t*(u_x*sin(theta) - u_y*cos(theta))/(l*m) + alpha &
      *u_t + lambda_theta*omega + lambda_v_x*(-1.0d0/2.0d0*A*C_d*rho*vx &
      *sqrt(vx**2 + vy**2) + T*u_t*u_x)/m + lambda_v_y*(-1.0d0/2.0d0*A* &
      C_d*rho*vy*sqrt(vx**2 + vy**2) + T*u_t*u_y - g*m)/m + lambda_x*vx &
      + lambda_y*vy + u_t**2*(-alpha + 1)

end subroutine

subroutine pontryagin(x, y, vx, vy, theta, omega, lambda_x, lambda_y, &
      lambda_v_x, lambda_v_y, lambda_theta, lambda_omega, alpha, &
      g, T, m, l, rho, C_d, A, u)
implicit none
REAL*8, intent(in) :: x
REAL*8, intent(in) :: y
REAL*8, intent(in) :: vx
REAL*8, intent(in) :: vy
REAL*8, intent(in) :: theta
REAL*8, intent(in) :: omega
REAL*8, intent(in) :: lambda_x
REAL*8, intent(in) :: lambda_y
REAL*8, intent(in) :: lambda_v_x
REAL*8, intent(in) :: lambda_v_y
REAL*8, intent(in) :: lambda_theta
REAL*8, intent(in) :: lambda_omega
REAL*8, intent(in) :: alpha
REAL*8, intent(in) :: g
REAL*8, intent(in) :: T
REAL*8, intent(in) :: m
REAL*8, intent(in) :: l
REAL*8, intent(in) :: rho
REAL*8, intent(in) :: C_d
REAL*8, intent(in) :: A
REAL*8, intent(out), dimension(1:3, 1:1) :: u

u(1, 1) = (1.0d0/2.0d0)*(-T*l**3*lambda_v_x**2*sqrt(l**4*lambda_v_x**2 + &
      l**4*lambda_v_y**2 + 24*l**2*lambda_omega*lambda_v_x*sin(theta) - &
      24*l**2*lambda_omega*lambda_v_y*cos(theta) + 144*lambda_omega**2 &
      ) - T*l**3*lambda_v_y**2*sqrt(l**4*lambda_v_x**2 + l**4* &
      lambda_v_y**2 + 24*l**2*lambda_omega*lambda_v_x*sin(theta) - 24*l &
      **2*lambda_omega*lambda_v_y*cos(theta) + 144*lambda_omega**2) - 6 &
      *T*l**2*lambda_omega*lambda_v_x*sqrt(l**4*lambda_v_x**2 + l**4* &
      lambda_v_y**2 + 24*l**2*lambda_omega*lambda_v_x*sin(theta) - 24*l &
      **2*lambda_omega*lambda_v_y*cos(theta) + 144*lambda_omega**2)*sin &
      (theta) + 6*T*l**2*lambda_omega*lambda_v_y*sqrt(l**4*lambda_v_x** &
      2 + l**4*lambda_v_y**2 + 24*l**2*lambda_omega*lambda_v_x*sin( &
      theta) - 24*l**2*lambda_omega*lambda_v_y*cos(theta) + 144* &
      lambda_omega**2)*cos(theta) - 12*T*l*lambda_omega*lambda_v_x*sqrt &
      (l**4*lambda_v_x**2 + l**4*lambda_v_y**2 + 24*l**2*lambda_omega* &
      lambda_v_x*sin(theta) - 24*l**2*lambda_omega*lambda_v_y*cos(theta &
      ) + 144*lambda_omega**2)*sin(theta) + 12*T*l*lambda_omega* &
      lambda_v_y*sqrt(l**4*lambda_v_x**2 + l**4*lambda_v_y**2 + 24*l**2 &
      *lambda_omega*lambda_v_x*sin(theta) - 24*l**2*lambda_omega* &
      lambda_v_y*cos(theta) + 144*lambda_omega**2)*cos(theta) - 72*T* &
      lambda_omega**2*sqrt(l**4*lambda_v_x**2 + l**4*lambda_v_y**2 + 24 &
      *l**2*lambda_omega*lambda_v_x*sin(theta) - 24*l**2*lambda_omega* &
      lambda_v_y*cos(theta) + 144*lambda_omega**2) + alpha*l**5* &
      lambda_v_x**2*m + alpha*l**5*lambda_v_y**2*m + 24*alpha*l**3* &
      lambda_omega*lambda_v_x*m*sin(theta) - 24*alpha*l**3*lambda_omega &
      *lambda_v_y*m*cos(theta) + 144*alpha*l*lambda_omega**2*m)/(l*m*( &
      alpha*(l**2*lambda_v_x + 12*lambda_omega*sin(theta))**2 + alpha*( &
      l**2*lambda_v_y - 12*lambda_omega*cos(theta))**2 - (l**2* &
      lambda_v_x + 12*lambda_omega*sin(theta))**2 - (l**2*lambda_v_y - &
      12*lambda_omega*cos(theta))**2))
u(2, 1) = -(l**2*lambda_v_x + 12*lambda_omega*sin(theta))/sqrt((l**2* &
      lambda_v_x + 12*lambda_omega*sin(theta))**2 + (l**2*lambda_v_y - &
      12*lambda_omega*cos(theta))**2)
u(3, 1) = (-l**2*lambda_v_y + 12*lambda_omega*cos(theta))/sqrt((l**2* &
      lambda_v_x + 12*lambda_omega*sin(theta))**2 + (l**2*lambda_v_y - &
      12*lambda_omega*cos(theta))**2)

end subroutine

subroutine eom_fullstate(x, y, vx, vy, theta, omega, lambda_x, &
      lambda_y, lambda_v_x, lambda_v_y, lambda_theta, &
      lambda_omega, u_t, u_x, u_y, g, T, m, l, rho, C_d, A, dfs)
implicit none
REAL*8, intent(in) :: x
REAL*8, intent(in) :: y
REAL*8, intent(in) :: vx
REAL*8, intent(in) :: vy
REAL*8, intent(in) :: theta
REAL*8, intent(in) :: omega
REAL*8, intent(in) :: lambda_x
REAL*8, intent(in) :: lambda_y
REAL*8, intent(in) :: lambda_v_x
REAL*8, intent(in) :: lambda_v_y
REAL*8, intent(in) :: lambda_theta
REAL*8, intent(in) :: lambda_omega
REAL*8, intent(in) :: u_t
REAL*8, intent(in) :: u_x
REAL*8, intent(in) :: u_y
REAL*8, intent(in) :: g
REAL*8, intent(in) :: T
REAL*8, intent(in) :: m
REAL*8, intent(in) :: l
REAL*8, intent(in) :: rho
REAL*8, intent(in) :: C_d
REAL*8, intent(in) :: A
REAL*8, intent(out), dimension(1:12, 1:1) :: dfs

dfs(1, 1) = vx
dfs(2, 1) = vy
dfs(3, 1) = (-1.0d0/2.0d0*A*C_d*rho*vx*sqrt(vx**2 + vy**2) + T*u_t*u_x)/ &
      m
dfs(4, 1) = (-1.0d0/2.0d0*A*C_d*rho*vy*sqrt(vx**2 + vy**2) + T*u_t*u_y - &
      g*m)/m
dfs(5, 1) = omega
dfs(6, 1) = 6*T*u_t*(u_x*sin(theta) - u_y*cos(theta))/(l*m)
dfs(7, 1) = 0
dfs(8, 1) = 0
dfs(9, 1) = (1.0d0/2.0d0)*A*C_d*lambda_v_y*rho*vx*vy/(m*sqrt(vx**2 + vy &
      **2)) - lambda_v_x*(-1.0d0/2.0d0*A*C_d*rho*vx**2/sqrt(vx**2 + vy &
      **2) - 1.0d0/2.0d0*A*C_d*rho*sqrt(vx**2 + vy**2))/m - lambda_x
dfs(10, 1) = (1.0d0/2.0d0)*A*C_d*lambda_v_x*rho*vx*vy/(m*sqrt(vx**2 + vy &
      **2)) - lambda_v_y*(-1.0d0/2.0d0*A*C_d*rho*vy**2/sqrt(vx**2 + vy &
      **2) - 1.0d0/2.0d0*A*C_d*rho*sqrt(vx**2 + vy**2))/m - lambda_y
dfs(11, 1) = -6*T*lambda_omega*u_t*(u_x*cos(theta) + u_y*sin(theta))/(l* &
      m)
dfs(12, 1) = -lambda_theta

end subroutine

subroutine eom_fullstate_jac(x, y, vx, vy, theta, omega, lambda_x, &
      lambda_y, lambda_v_x, lambda_v_y, lambda_theta, &
      lambda_omega, u_t, u_x, u_y, g, T, m, l, rho, C_d, A, ddfs)
implicit none
REAL*8, intent(in) :: x
REAL*8, intent(in) :: y
REAL*8, intent(in) :: vx
REAL*8, intent(in) :: vy
REAL*8, intent(in) :: theta
REAL*8, intent(in) :: omega
REAL*8, intent(in) :: lambda_x
REAL*8, intent(in) :: lambda_y
REAL*8, intent(in) :: lambda_v_x
REAL*8, intent(in) :: lambda_v_y
REAL*8, intent(in) :: lambda_theta
REAL*8, intent(in) :: lambda_omega
REAL*8, intent(in) :: u_t
REAL*8, intent(in) :: u_x
REAL*8, intent(in) :: u_y
REAL*8, intent(in) :: g
REAL*8, intent(in) :: T
REAL*8, intent(in) :: m
REAL*8, intent(in) :: l
REAL*8, intent(in) :: rho
REAL*8, intent(in) :: C_d
REAL*8, intent(in) :: A
REAL*8, intent(out), dimension(1:12, 1:12) :: ddfs

ddfs(1, 1) = 0
ddfs(2, 1) = 0
ddfs(3, 1) = 0
ddfs(4, 1) = 0
ddfs(5, 1) = 0
ddfs(6, 1) = 0
ddfs(7, 1) = 0
ddfs(8, 1) = 0
ddfs(9, 1) = 0
ddfs(10, 1) = 0
ddfs(11, 1) = 0
ddfs(12, 1) = 0
ddfs(1, 2) = 0
ddfs(2, 2) = 0
ddfs(3, 2) = 0
ddfs(4, 2) = 0
ddfs(5, 2) = 0
ddfs(6, 2) = 0
ddfs(7, 2) = 0
ddfs(8, 2) = 0
ddfs(9, 2) = 0
ddfs(10, 2) = 0
ddfs(11, 2) = 0
ddfs(12, 2) = 0
ddfs(1, 3) = 1
ddfs(2, 3) = 0
ddfs(3, 3) = (-1.0d0/2.0d0*A*C_d*rho*vx**2/sqrt(vx**2 + vy**2) - 1.0d0/ &
      2.0d0*A*C_d*rho*sqrt(vx**2 + vy**2))/m
ddfs(4, 3) = -1.0d0/2.0d0*A*C_d*rho*vx*vy/(m*sqrt(vx**2 + vy**2))
ddfs(5, 3) = 0
ddfs(6, 3) = 0
ddfs(7, 3) = 0
ddfs(8, 3) = 0
ddfs(9, 3) = -1.0d0/2.0d0*A*C_d*lambda_v_y*rho*vx**2*vy/(m*(vx**2 + vy** &
      2)**(3.0d0/2.0d0)) + (1.0d0/2.0d0)*A*C_d*lambda_v_y*rho*vy/(m* &
      sqrt(vx**2 + vy**2)) - lambda_v_x*((1.0d0/2.0d0)*A*C_d*rho*vx**3/ &
      (vx**2 + vy**2)**(3.0d0/2.0d0) - 3.0d0/2.0d0*A*C_d*rho*vx/sqrt(vx &
      **2 + vy**2))/m
ddfs(10, 3) = -1.0d0/2.0d0*A*C_d*lambda_v_x*rho*vx**2*vy/(m*(vx**2 + vy &
      **2)**(3.0d0/2.0d0)) + (1.0d0/2.0d0)*A*C_d*lambda_v_x*rho*vy/(m* &
      sqrt(vx**2 + vy**2)) - lambda_v_y*((1.0d0/2.0d0)*A*C_d*rho*vx*vy &
      **2/(vx**2 + vy**2)**(3.0d0/2.0d0) - 1.0d0/2.0d0*A*C_d*rho*vx/ &
      sqrt(vx**2 + vy**2))/m
ddfs(11, 3) = 0
ddfs(12, 3) = 0
ddfs(1, 4) = 0
ddfs(2, 4) = 1
ddfs(3, 4) = -1.0d0/2.0d0*A*C_d*rho*vx*vy/(m*sqrt(vx**2 + vy**2))
ddfs(4, 4) = (-1.0d0/2.0d0*A*C_d*rho*vy**2/sqrt(vx**2 + vy**2) - 1.0d0/ &
      2.0d0*A*C_d*rho*sqrt(vx**2 + vy**2))/m
ddfs(5, 4) = 0
ddfs(6, 4) = 0
ddfs(7, 4) = 0
ddfs(8, 4) = 0
ddfs(9, 4) = -1.0d0/2.0d0*A*C_d*lambda_v_y*rho*vx*vy**2/(m*(vx**2 + vy** &
      2)**(3.0d0/2.0d0)) + (1.0d0/2.0d0)*A*C_d*lambda_v_y*rho*vx/(m* &
      sqrt(vx**2 + vy**2)) - lambda_v_x*((1.0d0/2.0d0)*A*C_d*rho*vx**2* &
      vy/(vx**2 + vy**2)**(3.0d0/2.0d0) - 1.0d0/2.0d0*A*C_d*rho*vy/sqrt &
      (vx**2 + vy**2))/m
ddfs(10, 4) = -1.0d0/2.0d0*A*C_d*lambda_v_x*rho*vx*vy**2/(m*(vx**2 + vy &
      **2)**(3.0d0/2.0d0)) + (1.0d0/2.0d0)*A*C_d*lambda_v_x*rho*vx/(m* &
      sqrt(vx**2 + vy**2)) - lambda_v_y*((1.0d0/2.0d0)*A*C_d*rho*vy**3/ &
      (vx**2 + vy**2)**(3.0d0/2.0d0) - 3.0d0/2.0d0*A*C_d*rho*vy/sqrt(vx &
      **2 + vy**2))/m
ddfs(11, 4) = 0
ddfs(12, 4) = 0
ddfs(1, 5) = 0
ddfs(2, 5) = 0
ddfs(3, 5) = 0
ddfs(4, 5) = 0
ddfs(5, 5) = 0
ddfs(6, 5) = 6*T*u_t*(u_x*cos(theta) + u_y*sin(theta))/(l*m)
ddfs(7, 5) = 0
ddfs(8, 5) = 0
ddfs(9, 5) = 0
ddfs(10, 5) = 0
ddfs(11, 5) = -6*T*lambda_omega*u_t*(-u_x*sin(theta) + u_y*cos(theta))/( &
      l*m)
ddfs(12, 5) = 0
ddfs(1, 6) = 0
ddfs(2, 6) = 0
ddfs(3, 6) = 0
ddfs(4, 6) = 0
ddfs(5, 6) = 1
ddfs(6, 6) = 0
ddfs(7, 6) = 0
ddfs(8, 6) = 0
ddfs(9, 6) = 0
ddfs(10, 6) = 0
ddfs(11, 6) = 0
ddfs(12, 6) = 0
ddfs(1, 7) = 0
ddfs(2, 7) = 0
ddfs(3, 7) = 0
ddfs(4, 7) = 0
ddfs(5, 7) = 0
ddfs(6, 7) = 0
ddfs(7, 7) = 0
ddfs(8, 7) = 0
ddfs(9, 7) = -1
ddfs(10, 7) = 0
ddfs(11, 7) = 0
ddfs(12, 7) = 0
ddfs(1, 8) = 0
ddfs(2, 8) = 0
ddfs(3, 8) = 0
ddfs(4, 8) = 0
ddfs(5, 8) = 0
ddfs(6, 8) = 0
ddfs(7, 8) = 0
ddfs(8, 8) = 0
ddfs(9, 8) = 0
ddfs(10, 8) = -1
ddfs(11, 8) = 0
ddfs(12, 8) = 0
ddfs(1, 9) = 0
ddfs(2, 9) = 0
ddfs(3, 9) = 0
ddfs(4, 9) = 0
ddfs(5, 9) = 0
ddfs(6, 9) = 0
ddfs(7, 9) = 0
ddfs(8, 9) = 0
ddfs(9, 9) = -(-1.0d0/2.0d0*A*C_d*rho*vx**2/sqrt(vx**2 + vy**2) - 1.0d0/ &
      2.0d0*A*C_d*rho*sqrt(vx**2 + vy**2))/m
ddfs(10, 9) = (1.0d0/2.0d0)*A*C_d*rho*vx*vy/(m*sqrt(vx**2 + vy**2))
ddfs(11, 9) = 0
ddfs(12, 9) = 0
ddfs(1, 10) = 0
ddfs(2, 10) = 0
ddfs(3, 10) = 0
ddfs(4, 10) = 0
ddfs(5, 10) = 0
ddfs(6, 10) = 0
ddfs(7, 10) = 0
ddfs(8, 10) = 0
ddfs(9, 10) = (1.0d0/2.0d0)*A*C_d*rho*vx*vy/(m*sqrt(vx**2 + vy**2))
ddfs(10, 10) = -(-1.0d0/2.0d0*A*C_d*rho*vy**2/sqrt(vx**2 + vy**2) - &
      1.0d0/2.0d0*A*C_d*rho*sqrt(vx**2 + vy**2))/m
ddfs(11, 10) = 0
ddfs(12, 10) = 0
ddfs(1, 11) = 0
ddfs(2, 11) = 0
ddfs(3, 11) = 0
ddfs(4, 11) = 0
ddfs(5, 11) = 0
ddfs(6, 11) = 0
ddfs(7, 11) = 0
ddfs(8, 11) = 0
ddfs(9, 11) = 0
ddfs(10, 11) = 0
ddfs(11, 11) = 0
ddfs(12, 11) = -1
ddfs(1, 12) = 0
ddfs(2, 12) = 0
ddfs(3, 12) = 0
ddfs(4, 12) = 0
ddfs(5, 12) = 0
ddfs(6, 12) = 0
ddfs(7, 12) = 0
ddfs(8, 12) = 0
ddfs(9, 12) = 0
ddfs(10, 12) = 0
ddfs(11, 12) = -6*T*u_t*(u_x*cos(theta) + u_y*sin(theta))/(l*m)
ddfs(12, 12) = 0

end subroutine
