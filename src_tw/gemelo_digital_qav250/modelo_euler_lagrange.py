import numpy as np

class DroneModel:
    def  __init__(self, parameters):
        self.m = parameters["m"] # masa [kg] {la tenemos que sacar}
        self.l = parameters["l"] # longitud del brazo (del rotor a centro de masa) [m] {SOLIDWORKS}
        self.Ixx = parameters["Ixx"] # inercia en X {SOLIDWORKS}
        self.Iyy = parameters["Iyy"] # inercia en Y {SOLIDWORKS}
        self.Izz = parameters["Izz"] # inercia en Z {SOLIDWORKS}
        self.k = parameters["k"] # coeficiente de empuje [N·s^2/rad^2]  {la tenemos que sacar}
        self.b = parameters["b"] # coeficiente de arrastre [N·m·s^2/rad^2] {la tenemos que sacar}
        self.Ir = parameters["Ir"] # inercia del motor [kg·m^2] {datasheet del motor}
        self.Ax = parameters["Ax"] # arrastre aerodinamico en x [kg/s] {calcularlo de alguna manera o ponerle 0}
        self.Ay = parameters["Ay"] # arrastre aerodinamico en y [kg/s] {calcularlo de alguna manera o ponerle 0}
        self.Az = parameters["Az"] # arrastre aerodinamico en z [kg/s] {calcularlo de alguna manera o ponerle 0}

        self.g = 9.81 # gravedad [m/s^2]

        self.estado = np.zeros(12)
        print(f"DroneModel inicializado con: m = {self.m} kg, k = {self.k} N·s^2/rad^2")
    
    def calcular_fuerzas_torques(self, omega1, omega2, omega3, omega4):
        """Ecuaciones 6,7,8 de luukkonen adaptadas a configuración en X y mapeo PX4.
        Mapeo de motores:
          1: Frontal derecho   (CCW) -> x > 0, y < 0
          2: Trasero izquierdo (CCW) -> x < 0, y > 0
          3: Frontal izquierdo (CW)  -> x > 0, y > 0
          4: Trasero derecho   (CW)  -> x < 0, y < 0
        """
        # empuje de cada rotor: f_i = k * omega_i^2 (6)
        f1 = self.k * omega1**2 
        f2 = self.k * omega2**2
        f3 = self.k * omega3**2
        f4 = self.k * omega4**2

        # empuje total 
        T = f1 + f2 + f3 + f4 # (7)

        # torques en configuración X (el brazo efectivo para roll/pitch es l / sqrt(2))
        factor_l = self.l / np.sqrt(2.0)
        
        # Roll: diferencia entre izquierda (2, 3) y derecha (1, 4)
        tau_phi = factor_l * (-f1 + f2 + f3 - f4)
        
        # Pitch: diferencia entre trasero (2, 4) y frontal (1, 3) (trasero aumenta pitch en Luukkonen)
        tau_theta = factor_l * (-f1 + f2 - f3 + f4)
        
        # Yaw: diferencia entre rotores CCW (1, 2) y CW (3, 4)
        tau_psi = self.b * (omega1**2 + omega2**2 - omega3**2 - omega4**2)

        # Suma giroscópica de los rotores
        omega_G = omega1 + omega2 - omega3 - omega4
        return T, tau_phi, tau_theta, tau_psi, omega_G
    
    def W_eta(self, phi, theta):
        """Ecuacion 4 de luukkonen"""
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctht, stht = np.cos(theta), np.sin(theta)

        W = np.array([
            [1, 0, -stht],
            [0, cphi, ctht*sphi],
            [0, -sphi, ctht*cphi]
        ])
        return W
    
    def W_eta_inv(self, phi, theta):
        """Ecuacion 4 de luukkonen"""
        sphi, cphi = np.sin(phi), np.cos(phi)
        ctht, ttht = np.cos(theta), np.tan(theta)

        W_inv = np.array([
            [1, sphi*ttht, cphi*ttht],
            [0, cphi, -sphi],
            [0, sphi/ctht, cphi/ctht]
        ])
        return W_inv
    
    def Jacobiano_inercia(self, phi, theta):
        """ecuacion 16 de luukkonen"""
        I_diagonal = np.diag([self.Ixx, self.Iyy, self.Izz])
        W = self.W_eta(phi,theta)
        J = W.T @ I_diagonal @ W
        return J
    
    def Coriolis_C(self, phi, theta, phi_dot, theta_dot, psi_dot):
        sphi, cphi = np.sin(phi), np.cos(phi)
        stht, ctht = np.sin(theta), np.cos(theta)

        Ixx = self.Ixx
        Iyy = self.Iyy
        Izz = self.Izz

        #CHECAR QUE NO HAYA ESCRITO MAL ESTO D:
        C11 = 0
        C12 = ((Iyy - Izz) * ((theta_dot * cphi * sphi) + (psi_dot * sphi**2 * ctht))) + ((Izz - Iyy) * (psi_dot * cphi**2 * ctht)) - (Ixx * psi_dot * ctht)
        C13 = (Izz - Iyy) * (psi_dot * cphi * sphi * ctht**2)
        C21 = ((Izz - Iyy) * ((theta_dot * cphi * sphi) + (psi_dot * sphi * ctht))) + ((Iyy - Izz) * (psi_dot * cphi**2 * ctht)) + (Ixx * psi_dot * ctht)
        C22 = (Izz - Iyy) * (phi_dot * cphi * sphi)
        C23 = (-Ixx * psi_dot * stht * ctht) + (Iyy * psi_dot * sphi**2 * stht * ctht) + (Izz * psi_dot * cphi**2 * stht * ctht)
        C31 = ((Iyy - Izz) * (psi_dot * ctht**2 * sphi * cphi)) - (Ixx * theta_dot * ctht)
        C32 = ((Izz - Iyy) * ((theta_dot * cphi * sphi * stht) + (phi_dot * sphi**2 * ctht))) + ((Iyy - Izz) * (phi_dot * cphi**2 * ctht)) + (Ixx * psi_dot * stht * ctht) - (Iyy * psi_dot * sphi**2 * stht * ctht) - (Izz * psi_dot * cphi**2 * stht * ctht)
        C33 = ((Iyy - Izz) * (phi_dot * cphi * sphi * ctht**2)) - (Iyy * theta_dot * sphi**2 * ctht * stht) - (Izz * theta_dot * cphi**2 * ctht * stht) + (Ixx * theta_dot * ctht * stht)

        C = np.array([
            [C11, C12, C13],
            [C21, C22, C23],
            [C31, C32, C33]
        ])
        return C
    
    def derivadas_estado(self, estado, T, tau_phi, tau_theta, tau_psi, omega_G):
        """Ecuaciones 20 y 21 de Luukkonen"""
        #extraemos los datos de estado actual
        x , y, z = estado[0], estado[1], estado[2]
        phi, theta, psi = estado[3], estado[4], estado[5]
        dx, dy, dz = estado[6], estado[7], estado[8]
        dphi, dtheta, dpsi = estado[9], estado[10], estado[11]

        # DINAMICA TRASLACIONAL (21)
        sphi, cphi,  = np.sin(phi), np.cos(phi)
        stht, ctht = np.sin(theta), np.cos(theta)
        spsi, cpsi = np.sin(psi), np.cos(psi)

        #direccion de empuje en frame inercial
        thrust_dir_x = (cpsi * stht * cphi) + (spsi * sphi)
        thrust_dir_y = (spsi * stht * cphi) - (cpsi * sphi)
        thrust_dir_z = ctht * cphi

        #Aceleraciones
        ddx = ((T/self.m) * (thrust_dir_x)) - ((self.Ax/self.m) * dx)
        ddy = ((T/self.m) * (thrust_dir_y)) - ((self.Ay/self.m) * dy)
        ddz = ((T/self.m) * (thrust_dir_z)) - ((self.Az/self.m) * dz) - self.g

        #DINAMICA ROTACIONAL
        tau_B = np.array([tau_phi, tau_theta, tau_psi])
        eta_dot = np.array([dphi, dtheta, dpsi])

        J = self.Jacobiano_inercia(phi, theta)
        C = self.Coriolis_C(phi, theta, dphi, dtheta, dpsi)

        nu = self.W_eta(phi, theta) @ eta_dot
        gyro_body = self.Ir * np.cross(nu, np.array([0,0,1])) * omega_G
        gyro_euler = self.W_eta_inv(phi, theta) @ gyro_body

        eta_ddot = np.linalg.solve(J, tau_B - C @ eta_dot - gyro_euler)

        #vector de derivadas
        d_estado = np.array([
            dx, dy, dz, #velocidades
            dphi, dtheta, dpsi, #velocidades angulares
            ddx, ddy, ddz, # aceleraciones 
            eta_ddot[0], eta_ddot[1], eta_ddot[2] #aceleraciones angulares
        ])

        return d_estado
    
    def paso_rk4(self, dt, T, tau_phi, tau_theta, tau_psi, omega_G):
        args = (T, tau_phi, tau_theta, tau_psi, omega_G)

        k1 = self.derivadas_estado(self.estado, *args)
        k2 = self.derivadas_estado(self.estado + dt/2*k1, *args)
        k3 = self.derivadas_estado(self.estado + dt/2*k2, *args)
        k4 = self.derivadas_estado(self.estado + dt*k3, *args)

        return self.estado + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    
    def actualizar(self, omega1, omega2, omega3, omega4, dt):
        """Actualiza el estado del dron"""
        # 1. Calcular fuerzas y torques
        T, tau_phi, tau_tht, tau_psi, omega_G = self.calcular_fuerzas_torques(
            omega1, omega2, omega3, omega4
        )

        # 2. Integrar con RK4
        self.estado = self.paso_rk4(dt, T, tau_phi, tau_tht, tau_psi, omega_G)

        # 3. Retornar estado actualizado
        return self.estado
    
    def get_estado(self):
        """Retorna una copia del estado actual"""
        return self.estado.copy()
    
    def reset(self):
        """Resetea el estado a cero"""
        self.estado = np.zeros(12)
        print("Estado del drone reseteado a cero")
