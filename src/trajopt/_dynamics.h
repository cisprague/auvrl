
interface
subroutine lagrangian(u_t, u_x, u_y, alpha, L)
implicit none
REAL*8, intent(in) :: u_t
REAL*8, intent(in) :: u_x
REAL*8, intent(in) :: u_y
REAL*8, intent(in) :: alpha
REAL*8, intent(out) :: L
end subroutine
end interface
interface
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
end subroutine
end interface
interface
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
end subroutine
end interface
interface
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
end subroutine
end interface
interface
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
end subroutine
end interface

