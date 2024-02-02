import math
def circuit_properties(circuit_type, Xc, Xl, Z, Y, phi, angular_f, resonant_f, quality_factor, resonant_ang_f):
    formatted_Xc = format_values_r(abs(Xc))
    formatted_xl = format_values_r(abs(Xl))
    formatted_Z = format_values_r(abs(Z))
    formatted_resonant_f = format_values_f(abs(resonant_f))
    print("{} Circuit:".format(circuit_type))
    if circuit_type == "Series RC" or circuit_type == "Parallel RC":
        print("Angular frequency = {} rad/s ".format(angular_f))
        print("Capacitive reactance(Xc) = {} ".format(formatted_Xc))
        print("Total Impedance = {}".format(formatted_Z))
        print("Admittance = {:.6f} siemens".format(abs(Y)))
        print("Phase difference = {:.6f} degrees".format(phi * 180 / math.pi))
        print("Phase difference = {:.6f} rad".format(phi))
    elif circuit_type == "Series RL" or circuit_type == "Parallel RL":
        print("Angular frequency = {} rad/s ".format(angular_f))
        print("Inductive reactance(Xl) = {}".format(formatted_xl))
        print("Total Impedance = {}".format(formatted_Z))
        print("Admittance = {:.6f} siemens".format(abs(Y)))
        print("Phase difference = {:.6f} degrees".format(phi * 180 / math.pi))
        print("Phase difference = {:.6f} rad".format(phi))
    elif circuit_type == "Series RLC" or circuit_type == "Parallel RLC":
        print("Angular frequency = {} rad/s ".format(angular_f))
        print("Capacitive reactance(Xc) = {} ".format(formatted_Xc))
        print("Inductive reactance(Xl) = {}".format(formatted_xl))
        print("Total Impedance = {}".format(formatted_Z))
        print("Admittance = {:.6f} siemens".format(abs(Y)))
        print("Phase difference = {:.6f} degrees".format(phi * 180 / math.pi))
        print("Phase difference = {:.6f} rad".format(phi))
        print("Quality factor: {}".format(quality_factor))
        print("Resonant frequency = {} ".format(formatted_resonant_f))
        print("Resonant angular frequency = {:.6f} rad/s".format(resonant_ang_f))
    else:
        print("If you see this David made an oopsie")

def User_guide():
    print("Valid units:")
    print("mOhm,Ohm,kOhm,MOhm")
    print("mHz,Hz,kHz,MHz,GHz")
    print("H,mH,uH")
    print("F,mF,uF,nF,pF")
    print("Enjoy!")

calculate_omega = lambda f:2*math.pi*f
calculate_Xc = lambda C, f:1/((calculate_omega(f))*C)
calculate_Xl = lambda L, f:(calculate_omega(f))*L
calculate_impedance_S_RC = lambda R, Xc: math.sqrt((R ** 2) + (1 / (2 * math.pi * f * C)**2))
calculate_impedance_P_RC = lambda R, Xc: R*Xc/math.sqrt(R**2 + Xc**2)
calculate_impedance_S_RL = lambda R, Xl: math.sqrt(R ** 2 + Xl ** 2)
calculate_impedance_P_RL = lambda R, Xl: math.sqrt(1 / ((1 / R ** 2) + (1 / Xl ** 2)))
calculate_impedance_P_RLC = lambda R, Xc, Xl: 1 / math.sqrt((1 / R ** 2) + ((1 / Xl) - (1 / Xc))**2)
calculate_impedance_S_RLC = lambda R, Xc, Xl: math.sqrt(R **2 + (Xl - Xc)**2)
calculate_admittance = lambda Z: 1/Z
calculate_phase_angle_S_RC = lambda R, C,: math.atan(-(1 / (R * (calculate_omega(f)) * C)))
calculate_phase_angle_P_RC = lambda R, C,: math.atan(-(calculate_omega(f) * C * R))
calculate_phase_angle_S_RL = lambda R, L : math.atan(calculate_omega(f) * L / R)
calculate_phase_angle_P_RL = lambda R, L : math.atan(R / (calculate_omega(f) * L))
calculate_phase_angle_P_RLC = lambda R, Xc, Xl : math.atan((R / Xl) - (R / Xc))
calculate_phase_angle_S_RLC = lambda R, Xc, Xl: math.atan((Xl - Xc) / R)
calculate_AngularFrequency = lambda f : 2*math.pi*f
calculate_resonant_frequency = lambda L, C : 1 / (2*math.pi*math.sqrt(L*C))
calculate_quality_factor_S_RLC = lambda R, L, C : (1/R)*math.sqrt(L/C)
calculate_quality_factor_P_RLC = lambda R, L, C : R*math.sqrt(C/L)
calculate_resonant_angular_frequency = lambda L, C : 1 /math.sqrt(L*C)
def parallel_rc_calc(R, C, f,):
    angular_f = calculate_AngularFrequency(f)
    Xc = calculate_Xc(C, f)
    Z = calculate_impedance_P_RC(R, Xc)
    Y = calculate_admittance(Z)
    phi = calculate_phase_angle_P_RC(R, C)
    circuit_properties("Parallel RC", Xc, 0, Z, Y, phi, angular_f, 0, 0, 0)
def series_rc_calc(R, C, f):
    angular_f = calculate_AngularFrequency(f)
    Xc = calculate_Xc(C, f)
    Z = calculate_impedance_S_RC(R, Xc)
    Y = calculate_admittance(Z)
    phi = calculate_phase_angle_S_RC(R, C)
    circuit_properties("Series RC", Xc, 0, Z, Y, phi, angular_f, 0, 0, 0)
def series_rl_calc(R, L, f):
    angular_f = calculate_AngularFrequency(f)
    Xl = calculate_Xl(L, f)
    Z = calculate_impedance_S_RL(R, Xl)
    Y = calculate_admittance(Z)
    phi = calculate_phase_angle_S_RL(R, L)
    circuit_properties("Series RL", 0, Xl, Z, Y, phi, angular_f, 0, 0, 0)
def parallel_rl_calc(R, L, f):
    angular_f = calculate_AngularFrequency(f)
    Xl = calculate_Xl(L, f)
    Z = calculate_impedance_P_RL(R, Xl)
    Y = calculate_admittance(Z)
    phi = calculate_phase_angle_P_RL(R, L)
    circuit_properties("Parallel RL", 0, Xl, Z, Y, phi, angular_f, 0, 0, 0)
def parallel_rlc_calc(R, C, f, L):
    angular_f = calculate_AngularFrequency(f)
    Xc = calculate_Xc(C, f)
    Xl = calculate_Xl(L, f)
    Z = calculate_impedance_P_RLC(R, Xc, Xl)
    Y = calculate_admittance(Z)
    phi = calculate_phase_angle_P_RLC(R, Xc, Xl)
    resonant_f = calculate_resonant_frequency(L, C)
    resonant_angular_f = calculate_resonant_angular_frequency(L, C)
    q_factor = calculate_quality_factor_P_RLC(R, L, C)
    circuit_properties("Parallel RLC", Xc, Xl, Z, Y, phi, angular_f, resonant_f, q_factor, resonant_angular_f)
def series_rlc_calc(R, C, f, L):
    angular_f = calculate_AngularFrequency(f)
    Xc = calculate_Xc(C, f)
    Xl = calculate_Xl(L, f)
    Z = calculate_impedance_S_RLC(R, Xc, Xl)
    Y = calculate_admittance(Z)
    phi = calculate_phase_angle_S_RLC(R, Xc, Xl)
    resonant_f = calculate_resonant_frequency(L, C)
    resonant_angular_f = calculate_resonant_angular_frequency(L, C)
    q_factor = calculate_quality_factor_S_RLC(R, L, C)
    circuit_properties("Series RLC", Xc, Xl, Z, Y, phi, angular_f, resonant_f, q_factor, resonant_angular_f)

def scan_for_help_intput(user_input):
    if user_input == "h":
        User_guide()
        return 1
def get_valid_unit(prompt, valid_units):
    while True:
        user_input = input(prompt)
        if scan_for_help_intput(user_input) == 1:
            continue
        if user_input in valid_units:
            return user_input
        else:
            print("Invalid unit!")
def get_valid_value(prompt):
    while True:
        user_input = input(prompt)
        if scan_for_help_intput(user_input) == 1:
            continue
        try:
            value = float(user_input)
            if value > 0:
                return value
            else:
                print("Plz enter value < 0.")
        except ValueError:
            print("Invalid Value!")
def convert_units(x, unit):
    if unit in ["Ohm", "Hz", "F", "H"]:
        return x
    elif unit in ["kOhm", "kHz"]:
        return x * 1000.0
    elif unit in ["MOhm", "MHz"]:
        return x * 1000000.0
    elif unit == "GHz":
        return x * 1000000000.0
    elif unit in ["mOhm", "mHz", "mF", "mH"]:
        return x * 0.001
    elif unit in ["uF", "uH"]:
        return x * 0.000001
    elif unit == "nF":
        return x * 0.000000001
    elif unit == "pF":
        return x * 0.000000000001
def format_values_r(x):
    if x >= 1e6:
        return "{:.6f} MOhm".format(x / 1e6)
    elif x >= 1e3:
        return "{:.6f} kOhm".format(x / 1e3)
    elif x < 1:
        return "{:.6f} mOhm".format(x / 0.001)
    else:
        return "{:.6f} Ohm".format(x)
def format_values_f(x):
    if x >= 1e6:
        return "{:.6f} MHz".format(x / 1e6)
    elif x >= 1e3:
        return "{:.6f} kHz".format(x / 1e3)
    elif x < 0.01:
        return "{:.6f} mHz".format(x / 0.001)
    else:
        return "{:.6f} Hz".format(x)
valid_unit_R = ["mOhm", "Ohm", "kOhm", "MOhm"]
valid_unit_f = ["mHz", "Hz", "kHz", "MHz", "GHz"]
valid_unit_L = ["H", "mH", "uH"]
valid_unit_C = ["F", "mF", "uF", "nF", "pF"]
valid_circuit_type = ["RL", "RC", "RLC"]
valid_circuit_kind = ["S", "P"]

print("Enter\"h\" for help")
while True:
    circuit_type = input("(RL,RC,or RLC):")
    if scan_for_help_intput(circuit_type) == 1:
        continue
    if circuit_type.upper() not in valid_circuit_type:
        print("Invalid circuit type.")
    else:
        break

while True:
    circuit_kind = input("(P or S):")
    if scan_for_help_intput(circuit_kind) == 1:
        continue
    if circuit_kind.upper() not in valid_circuit_kind:
        print("Invalid circuit type. ")
    else:
        break
if circuit_type.upper() == "RC" and circuit_kind.upper() == "S":
    R_unit = get_valid_unit("Enter r unit: ", valid_unit_R)
    R = get_valid_value("Enter r value: ")
    R = convert_units(R, R_unit)
    f_unit = get_valid_unit("Enter f unit: ", valid_unit_f)
    f = get_valid_value("Enter f value: ")
    f = convert_units(f, f_unit)
    C_unit = get_valid_unit("Enter c unit: ", valid_unit_C)
    C = get_valid_value("Enter c value: ")
    C = convert_units(C, C_unit)
    series_rc_calc(R, C, f)
elif circuit_type.upper() == "RC" and circuit_kind.upper() == "P":
    R_unit = get_valid_unit("Enter r unit: ", valid_unit_R)
    R = get_valid_value("Enter r value: ")
    R = convert_units(R, R_unit)
    f_unit = get_valid_unit("Enter f unit: ", valid_unit_f)
    f = get_valid_value("Enter f value: ")
    f = convert_units(f, f_unit)
    C_unit = get_valid_unit("Enter c unit: ", valid_unit_C)
    C = get_valid_value("Enter c value: ")
    C = convert_units(C, C_unit)
    parallel_rc_calc(R, C, f)
elif circuit_type.upper() == "RL" and circuit_kind.upper() == "S":
    R_unit = get_valid_unit("Enter r unit: ", valid_unit_R)
    R = get_valid_value("Enter r value: ")
    R = convert_units(R, R_unit)
    f_unit = get_valid_unit("Enter f unit: ", valid_unit_f)
    f = get_valid_value("Enter f value: ")
    f = convert_units(f, f_unit)
    L_unit = get_valid_unit("Enter L unit: ", valid_unit_L)
    L = get_valid_value("Enter L value: ")
    L = convert_units(L, L_unit)
    series_rl_calc(R, L, f)
elif circuit_type.upper() == "RL" and circuit_kind.upper() == "P":
    R_unit = get_valid_unit("Enter r unit: ", valid_unit_R)
    R = get_valid_value("Enter a r value: ")
    R = convert_units(R, R_unit)
    f_unit = get_valid_unit("Enter f unit: ", valid_unit_f)
    f = get_valid_value("Enter f value: ")
    f = convert_units(f, f_unit)
    L_unit = get_valid_unit("Enter L unit: ", valid_unit_L)
    L = get_valid_value("Enter L value: ")
    L = convert_units(L, L_unit)
    parallel_rl_calc(R, L, f)
elif circuit_type.upper() == "RLC" and circuit_kind.upper() == "S":
    R_unit = get_valid_unit("Enter r unit: ", valid_unit_R)
    R = get_valid_value("Enter r value: ")
    R = convert_units(R, R_unit)
    f_unit = get_valid_unit("Enter f unit: ", valid_unit_f)
    f = get_valid_value("Enter f value: ")
    f = convert_units(f, f_unit)
    L_unit = get_valid_unit("Enter L unit: ", valid_unit_L)
    L = get_valid_value("Enter L value: ")
    L = convert_units(L, L_unit)
    C_unit = get_valid_unit("Enter c unit: ", valid_unit_C)
    C = get_valid_value("Enter c value: ")
    C = convert_units(C, C_unit)
    series_rlc_calc(R, C, f, L)
else:
    R_unit = get_valid_unit("Enter r unit: ", valid_unit_R)
    R = get_valid_value("Enter r value: ")
    R = convert_units(R, R_unit)
    f_unit = get_valid_unit("Enter f unit: ", valid_unit_f)
    f = get_valid_value("Enter f value: ")
    f = convert_units(f, f_unit)
    L_unit = get_valid_unit("Enter L unit: ", valid_unit_L)
    L = get_valid_value("Enter L value: ")
    L = convert_units(L, L_unit)
    C_unit = get_valid_unit("Enter c unit: ", valid_unit_C)
    C = get_valid_value("Enter c value: ")
    C = convert_units(C, C_unit)
    parallel_rlc_calc(R, C, f, L)
