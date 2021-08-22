% Multi-source dispatch strategy - MatlabDispatch2021_v1

%%% Notes: the goal of this dispatch function is to use renewables to shave
%%% peak demand and save fuel as much as possible.
%%% This function is designed to be used together with HOMER Pro software. 
%%% The full function package include MatlabStart function, MatlabEnd function 
%%% and MatlabDispatch2021_v1 fucntion. To use these functions, select
%%% matlablink to be the controller, select the path where these functions
%%% are stored, and then enter the names of these functions.

%%% Contact: dlei7@wisc.edu

 
%%% Compatible model settings: 1 electric load(ac)& 
%%% 0/1 generator(ac), 0/1 pv(dc), 0/1 battery(dc), 0/1 wind turbine(ac).

%%% Variable Definitions: most of the variables are defined in the 
%%% comments in this functions. For other variables, see definition here: 
%%% https://www.homerenergy.com/products/pro/docs/latest/_listing_of_simulationstate.html


function [simulation_state, matlab_simulation_variables] = MatlabDispatch2021_v1(simulation_parameters, simulation_state, matlab_simulation_variables)

    % define a small enough number to acoount for system errors
    eps = 0.00001;
    
    % initiate a zero matrix for storing the output parameters, 
    % 3 rows for three choices and 14 columns for 14 variables 
    parameters = zeros(3,14);

    % column names: 
    % 1 - simulation_state.generators(1).power_setpoint
    generator_power_setpoint = 1;
    % 2 - simulation_state.converters(1).inverter_power_input
    inverter_power_input = 2;
    % 3 - simulation_state.converters(1).inverter_power_output
    inverter_power_output = 3;
    % 4 - simulation_state.converters(1).rectifier_power_input
    rectifier_power_input = 4;
    % 5 - simulation_state.converters(1).rectifier_power_output
    rectifier_power_output = 5;
    % 6 - simulation_state.batteries(1).power_setpoint
    battery_power_setpoint = 6;
    % 7 - simulation_state.primary_loads(1).load_served
    primary_load_served = 7;
    % 8 - simulation_state.ac_bus.excess_electricity
    ac_bus_excess_electricity = 8;
    % 9 - simulation_state.ac_bus.load_served
    ac_bus_load_served = 9;
    % 10 - simulation_state.ac_bus.capacity_requested
    ac_bus_operating_capacity_requested = 10;
    % 11 - simulation_state.ac_bus.operating_capacity_served
    ac_bus_operating_capacity_served = 11;
    % 12 - simulation_state.ac_bus.unmet_load
    ac_bus_unmet_load = 12;
    % 13 - simulation_state.ac_bus.capacity_shortage
    ac_bus_capacity_shortage = 13;
    % 14 - marginal cost ($/kWh)
    marginal_cost = 14;


    % We don't have dc load in green case. 
    % Here we set all related state variable to 0 with function ignore_dc.
    % See how ignore_dc works at the end of this function.
    simulation_state = ignore_dc(simulation_state);

    % Define rectifier and inverter efficiency
    if simulation_parameters.has_battery == true
        rect_efficiency = 95;
        inv_efficiency = 95;
    else
        rect_efficiency = 1;
        inv_efficiency = 1;
    end
    % Pass the efficiency value to parameter variables.
    simulation_parameters.converters(1).rectifier_efficiency = rect_efficiency;
    simulation_parameters.converters(1).inverter_efficiency = inv_efficiency;

    % Initialize battery and rectifier system
    simulation_state.batteries(1).power_setpoint = 0;
    simulation_state.converters(1).rectifier_power_input = 0;
    simulation_state.converters(1).rectifier_power_output = 0;

    % Initialize inverter system
    simulation_state.converters(1).inverter_power_output = 0;
    simulation_state.converters(1).inverter_power_input = 0; 

    %%  
    % Step 1: dispatch wind and then PV
    
    % Extract load value from load state variable defined by HOMER
    load_requested = simulation_state.ac_bus.load_requested;
      
    % WIND ENERGY
    % Determine available wind
    avail_wind_power = dispatch_wind(simulation_state, simulation_parameters);
    % Serve the load with wind - dispatched value bounded by load
    dispatched_wind_power = min(load_requested, avail_wind_power);
    net_load_after_wind = load_requested - dispatched_wind_power;
    excess_wind = avail_wind_power - dispatched_wind_power;
    % Initialize wind power used to charge battery
    wind_to_batteries = 0;

    % SOLAR PV ENERGY
    % Dispatch solar power to DC network using dispatch_solar function.
    % See how dispatch_solar works at the end of this function.
    [avail_solar_ac, simulation_state] = dispatch_solar(simulation_state, simulation_parameters, inv_efficiency);
    % Serve load with solar
    dispatched_solar_power = min(net_load_after_wind, avail_solar_ac);
    % Update inverter flow using increment_inverter_load function.
    % See how increment_inverter_load works at the end of this function.
    simulation_state = increment_inverter_load(simulation_state, dispatched_solar_power, inv_efficiency);
    net_load_after_solar = net_load_after_wind - dispatched_solar_power;
    excess_solar_ac = avail_solar_ac - dispatched_solar_power;
    excess_solar_dc = excess_solar_ac / (inv_efficiency/100);
    % Initialize solar power used to charge battery
    solar_to_batteries = 0;
      
    % Charge the battery using solar and then wind
    % Charge batteries with excess solar using charge_batteries_dc function.
    % See how charge_batteries_dc works at the end of this function.
    if (excess_solar_ac > 0)
        if (simulation_parameters.has_battery == true)
            [solar_to_batteries, simulation_state] = charge_batteries_dc(simulation_state, simulation_parameters, excess_solar_dc);
            excess_solar_dc = excess_solar_dc - solar_to_batteries;
            excess_solar_ac = excess_solar_dc * inv_efficiency/100;
        end
    end        
    % Charge batteries with excess wind
    % Charge batteries with excess wind using charge_batteries_ac function.
    % See how charge_batteries_ac works at the end of this function.
    if (excess_wind > 0)
        if simulation_parameters.has_converter == true && simulation_parameters.has_battery == true
            [wind_to_batteries, simulation_state] = charge_batteries_ac(simulation_state, simulation_parameters, excess_wind, rect_efficiency);
            excess_wind = excess_wind - wind_to_batteries;
        end
    end
    % Update rectifier flow using increment_rectifier_load function.
    % See how increment_rectifier_load works at the end of this function.
    simulation_state = increment_rectifier_load(simulation_state, wind_to_batteries, rect_efficiency);
               
    %%
    % Step 2: Decide how to meet net load with generator and battery by
    % minimizing marginal cost ($/kWh).
    % Here we consier three options: 
      % option 1: discharge battery to meet the net load as much as possible, use the generator to fill the remaining
         % marginal cost ($/kWh) = average cost of charging + battery wear cost + value of stored energy +(cost of using the generator + cost of unmet load)
      % option 2: ramp up the generator to follow the net load, store the excess eletricity if net load is lower than minimum output
         % marginal cost ($/kWh) = cost of using the generator + (average cost of charging + battery wear cost - value of stored energy + cost of unmet load)
      % option 3: ramp up the generator as much as possible
         % marginal cost ($/kWh) = cost of using the generator + average cost of charging + battery wear cost - value of stored energy

    % Store initial value seperately for later reinitializing use (option = 2&3)
    inverter_output_temp = simulation_state.converters(1).inverter_power_output;
    rectifier_output_temp = simulation_state.converters(1).rectifier_power_output;
    battery_power_setpoint_temp = simulation_state.batteries(1).power_setpoint;

    % Loop through the three options
    for option = 1:3
        % Initialize intermediate variables
        dispatched_battery_ac = 0;
        battery_charge_power = 0;
        dispatched_battery_ac = 0;
        dispatched_generator = 0;
        excess_generator = 0;
        ac_to_batteries = 0;
        ac_charging_power = 0;
        charging_power_ac_to_dc = 0;
        remaining_load = net_load_after_solar;
        % Pass on converter flow from intermediate variables to output
        % state variables
        simulation_state.converters(1).inverter_power_output = inverter_output_temp;
        simulation_state.converters(1).rectifier_power_output = rectifier_output_temp;
        simulation_state.batteries(1).power_setpoint = battery_power_setpoint_temp;           
            
        %% Option 1: dispatch batteries first
        if (option == 1)
            avail_battery_ac = dispatch_batteries(simulation_state, simulation_parameters, remaining_load, inv_efficiency);
            % Serve load with battery bounded by available battery power
            % and remaning load requirement.
            dispatched_battery_ac = min(remaining_load, avail_battery_ac);
            simulation_state.batteries(1).power_setpoint = simulation_state.batteries(1).power_setpoint - dispatched_battery_ac/(inv_efficiency/100);
            % Update inverter flow using increment_inverter_load function.
            % See how increment_inverter_load works at the end of this function.
            simulation_state = increment_inverter_load(simulation_state, dispatched_battery_ac, inv_efficiency);
        end
        
        % Update remaining load:
        % For option 1 - reduce load; for option 2 & 3 - no effect
        remaining_load = remaining_load - dispatched_battery_ac;            
           
        %% Option 3: charge battery as much as possible in addition to meeting load requirement
        % Determine maximum charge power based on battery charging state
        if (option == 3)
            if simulation_parameters.has_battery == true
                battery_charge_power = simulation_state.batteries(1).max_charge_power - simulation_state.batteries(1).power_setpoint;
            else
                battery_charge_power = 0;
            end
        end
        
        % Determine maximum demand for generator
        generator_demand = remaining_load + battery_charge_power;
           
        %% Dispatch generator based on different rules defined in the 3 options
        if (remaining_load > eps)
            if (option == 3)
                avail_generator_power = dispatch_generator(simulation_state, simulation_parameters, generator_demand, eps);
                dispatched_generator = avail_generator_power;
                excess_generator = max(0,dispatched_generator - remaining_load);
            else
                avail_generator_power = dispatch_generator(simulation_state, simulation_parameters, remaining_load, eps);
                dispatched_generator = min(remaining_load, avail_generator_power);
                excess_generator = avail_generator_power - dispatched_generator;
            end
        else
            if (option == 3)
                avail_generator_power = dispatch_generator(simulation_state, simulation_parameters, generator_demand, eps);
                dispatched_generator = avail_generator_power;
                excess_generator = max(0,dispatched_generator - remaining_load);
            else
                avail_generator_power = 0;
                dispatched_generator = min(remaining_load, avail_generator_power);
                excess_generator = avail_generator_power - dispatched_generator;
            end
        end

        % Serve load with generator and update remaining load
        remaining_load = max(0,remaining_load - dispatched_generator);                                               
            
        %% Dispatch batteries after generator (option 2 & 3)
        if (remaining_load > eps)
            avail_battery_ac = dispatch_batteries(simulation_state, simulation_parameters, remaining_load, inv_efficiency);
            % Serve load with battery
            dispatched_battery_ac = min(remaining_load, avail_battery_ac);
            simulation_state.batteries(1).power_setpoint = simulation_state.batteries(1).power_setpoint - dispatched_battery_ac/(inv_efficiency/100);
            simulation_state = increment_inverter_load(simulation_state, dispatched_battery_ac, inv_efficiency);
        end            
        
        % Charge batter with excess generator
        if (excess_generator > eps)
            ac_to_batteries = get_battery_ac_charge_power(simulation_state, simulation_parameters, excess_generator, rect_efficiency);
            [ac_charging_power, simulation_state] = charge_batteries_ac(simulation_state, simulation_parameters, ac_to_batteries, rect_efficiency);
            excess_generator = excess_generator - ac_to_batteries;
            charging_power_ac_to_dc = ac_charging_power * rect_efficiency/100;
        end

        % Pass on values to the matrix
        parameters(option,generator_power_setpoint) = avail_generator_power;
        parameters(option,battery_power_setpoint) = simulation_state.batteries(1).power_setpoint;
        parameters(option,inverter_power_output) = simulation_state.converters(1).inverter_power_output;
        parameters(option,inverter_power_input) = parameters(option, inverter_power_output) / (inv_efficiency/100);
        parameters(option,rectifier_power_input) = simulation_state.converters(1).rectifier_power_input;
        parameters(option,rectifier_power_output) = simulation_state.converters(1).rectifier_power_input * (rect_efficiency/100);         
        parameters(option,primary_load_served) = dispatched_wind_power + dispatched_solar_power + dispatched_battery_ac + dispatched_generator - excess_generator;
        parameters(option,ac_bus_excess_electricity) = excess_solar_ac + excess_wind + excess_generator;
        parameters(option,ac_bus_load_served) = parameters(option,primary_load_served);
        parameters(option,ac_bus_operating_capacity_requested) = parameters(option,primary_load_served) * ... 
            (1 + simulation_parameters.operating_reserve.timestep_requirement/100);
        parameters(option,ac_bus_unmet_load) = simulation_state.ac_bus.load_requested - parameters(option,primary_load_served);
        parameters(option,ac_bus_operating_capacity_served) = parameters(option,ac_bus_load_served) + parameters(option,ac_bus_excess_electricity);
        parameters(option,ac_bus_capacity_shortage) = max(0, parameters(option,ac_bus_operating_capacity_requested)- parameters(option,ac_bus_operating_capacity_served));
                 
    end

    %% Calculate marginal cost for each option
    if  (simulation_parameters.has_battery == true && simulation_parameters.has_converter == true)
      parameters(:,marginal_cost) = ...
        (matlab_simulation_variables.generator_fuel_cost * parameters(:,generator_power_setpoint))/load_requested ...
      + (parameters(:,generator_power_setpoint)>0) * matlab_simulation_variables.generator_OM / load_requested ...
      + matlab_simulation_variables.electricity_price * parameters(:,ac_bus_unmet_load)/load_requested ...
      + simulation_state.batteries(1).energy_cost * parameters(:,inverter_power_output)/load_requested ...      % 0 if no batteries
      + simulation_parameters.batteries(1).wear_cost * parameters(:,inverter_power_output)/load_requested ...   % 0 if no batteries
      + simulation_parameters.batteries(1).wear_cost * parameters(:,rectifier_power_input)/load_requested  ...  % 0 if no batteries
      + matlab_simulation_variables.electricity_price * parameters(:,inverter_power_output)/load_requested ...  % 0 if no batteries
      - matlab_simulation_variables.electricity_price * parameters(:,rectifier_power_input)/load_requested;     % 0 if no batteries
    else
      parameters(:,marginal_cost) = ...
        (matlab_simulation_variables.generator_fuel_cost * parameters(:,generator_power_setpoint))/load_requested ...
      + (parameters(:,generator_power_setpoint)>0) * matlab_simulation_variables.generator_OM / load_requested ...
      + matlab_simulation_variables.electricity_price * parameters(:,ac_bus_unmet_load)/load_requested;
    end

    % Pass on cost values to the matrix
    costs = parameters(:,marginal_cost);
    
    % Record the option index that has the lowest cost
    indx_min_cost = find(costs == min(costs));
    % If there is a tie, always choose the one that charge the battery most
    if length(indx_min_cost)>1
        indx_min_cost = max(indx_min_cost);
    end


    % Pass on the values of the winner option to the state variables
    simulation_state.generators(1).power_setpoint          = parameters(indx_min_cost,generator_power_setpoint);
    simulation_state.converters(1).inverter_power_input    = parameters(indx_min_cost,inverter_power_input);
    simulation_state.converters(1).inverter_power_output   = parameters(indx_min_cost,inverter_power_output);
    simulation_state.converters(1).rectifier_power_input   = parameters(indx_min_cost,rectifier_power_input);
    simulation_state.converters(1).rectifier_power_output  = parameters(indx_min_cost,rectifier_power_output);
    simulation_state.batteries(1).power_setpoint           = parameters(indx_min_cost,battery_power_setpoint);
    simulation_state.primary_loads(1).load_served          = parameters(indx_min_cost,primary_load_served);
    simulation_state.ac_bus.excess_electricity             = parameters(indx_min_cost,ac_bus_excess_electricity);
    simulation_state.ac_bus.load_served                    = parameters(indx_min_cost,ac_bus_load_served);
    simulation_state.ac_bus.operating_capacity_served      = parameters(indx_min_cost,ac_bus_operating_capacity_served);
    simulation_state.ac_bus.unmet_load                     = parameters(indx_min_cost,ac_bus_unmet_load);
    simulation_state.ac_bus.capacity_shortage              = simulation_state.ac_bus.operating_capacity_requested - simulation_state.ac_bus.operating_capacity_served;

    

    
    
    
    
    
    
    
    
    
    
    
%% Intermediate functions

% ignore_dc
% initialize all dc state variables to zero
%
% inputs
% ------
% simulation_state : object holding current state of the simulation
%
% outputs
% ------
% simulation_state : object holding current state of the simulation
function simulation_state = ignore_dc(simulation_state)
    simulation_state.dc_bus.capacity_shortage = 0;
    simulation_state.dc_bus.excess_electricity = 0;
    simulation_state.dc_bus.load_served = 0;
    simulation_state.dc_bus.operating_capacity_served = 0;
    simulation_state.dc_bus.unmet_load = 0;
end

% increment_rectifier_load
% increment the power flowing through the rectifier on both the input and output
%
% assumes that dc_power is less than remaining capacity in rectifier
%
% inputs
% ------
% simulation_state : object holding current state of the simulation
% dc_power : the increment of DC power being delivered by the rectifier
% rect_efficiency : the AC->DC efficiency of the rectifier
%
% outputs
% ------
% simulation_state : object holding current state of the simulation
function simulation_state = increment_rectifier_load(simulation_state, dc_power, rect_efficiency)
    if simulation_parameters.has_converter == true
        simulation_state.converters(1).rectifier_power_output = simulation_state.converters(1).rectifier_power_output + dc_power;
        simulation_state.converters(1).rectifier_power_input = simulation_state.converters(1).rectifier_power_output / (rect_efficiency/100);
    else
        simulation_state.converters(1).rectifier_power_output = 0;
        simulation_state.converters(1).rectifier_power_input = 0;
    end
end

% increment_inverter_load
% increment the power flowing through the inverter on both the input and output
%
% assumes that ac_power is less than remaining capacity in inverter
%
% inputs
% ------
% simulation_state : object holding current state of the simulation
% ac_power: the increment of AC power being delivered by the inverter
% inv_efficiency : the DC->AC efficiency of the rectifier
%
% outputs
% ------
% simulation_state : object holding current state of the simulation
function simulation_state = increment_inverter_load(simulation_state, ac_power, inv_efficiency)
    if simulation_parameters.has_converter == true
        simulation_state.converters(1).inverter_power_output = simulation_state.converters(1).inverter_power_output + ac_power;
        simulation_state.converters(1).inverter_power_input = simulation_state.converters(1).inverter_power_output / (inv_efficiency/100); 
    else
        simulation_state.converters(1).inverter_power_output = 0;
        simulation_state.converters(1).inverter_power_input = 0; 
    end
end

% get_battery_dc_charge_power
% query the remaining DC charge available in the battery
% without changing the charge state
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% dc_power : DC power available to charge batteries
%
% outputs
% -------
% charge_power : DC power used to charge batteries
function charge_power = get_battery_dc_charge_power(simulation_state, simulation_parameters, dc_power)
    charge_power = 0;
    if simulation_parameters.has_battery == true
        max_avail_battery_charge_power = simulation_state.batteries(1).max_charge_power - simulation_state.batteries(1).power_setpoint;
        charge_power = min(max_avail_battery_charge_power, dc_power);
    end
end

% get_battery_ac_charge_power
% query the remaining AC charge available in the battery
% without changing the charge state
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% ac_power : DC power available to charge batteries
% rect_efficiency : the AC->DC efficiency of the rectifier
%
% outputs
% -------
% ac_charging_power : AC power used to charge batteries
function ac_charging_power_local = get_battery_ac_charge_power(simulation_state, simulation_parameters, ac_power, rect_efficiency)
    ac_charging_power_local = 0;
    if (simulation_parameters.has_battery == true && simulation_parameters.has_converter == true)
        dc_power = ac_power * rect_efficiency/100;
        max_avail_rectifier_output = simulation_parameters.converters(1).rectifier_capacity - simulation_state.converters(1).rectifier_power_output;
        dc_power_for_charging = min(dc_power, max_avail_rectifier_output);
        dc_charging_power = get_battery_dc_charge_power(simulation_state, simulation_parameters, dc_power_for_charging);
        ac_charging_power_local = dc_charging_power / (rect_efficiency/100);
    end
end

% charge_batteries_dc
% charge the batteries with DC power
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% dc_power : DC power available to charge batteries
%
% outputs
% -------
% simulation_state : object holding current state of simulation
% charge_power : DC power used to charge batteries
function [charge_power, simulation_state] = charge_batteries_dc(simulation_state, simulation_parameters, dc_power)
    charge_power = get_battery_dc_charge_power(simulation_state, simulation_parameters, dc_power);
    simulation_state.batteries(1).power_setpoint = simulation_state.batteries(1).power_setpoint + charge_power;
end

% charge_batteries_ac
% charge the batteries with AC power
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% ac_power : DC power available to charge batteries
% rect_efficiency : the AC->DC efficiency of the rectifier
%
% outputs
% -------
% simulation_state : object holding current state of simulation
% ac_charging_power : AC power used to charge batteries
function [ac_charging_power_local, simulation_state] = charge_batteries_ac(simulation_state, simulation_parameters, ac_power, rect_efficiency)
    dc_power = ac_power * rect_efficiency/100;
    if simulation_parameters.has_converter == true
        max_avail_rectifier_output = simulation_parameters.converters(1).rectifier_capacity - simulation_state.converters(1).rectifier_power_output;
    else
        max_avail_rectifier_output = 0;
    end

    dc_power_for_charging = min(dc_power, max_avail_rectifier_output);
    [dc_charging_power, simulation_state] = charge_batteries_dc(simulation_state, simulation_parameters, dc_power_for_charging);
    simulation_state = increment_rectifier_load(simulation_state, dc_charging_power, rect_efficiency);
    ac_charging_power_local = dc_charging_power / (rect_efficiency/100);
end

% dispatch_generator
% dispatch generator to deliver AC power
%
% query's the available power without changing the generator's state
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% power : AC power requested from generator
% eps
%
% outputs
% -------
% avail_generator_power : AC generator power available for dispatch
function avail_generator_power = dispatch_generator(simulation_state, simulation_parameters, power, eps)
    avail_generator_power = 0;
    if simulation_parameters.has_generator == true
        min_load = simulation_parameters.generators(1).minimum_load;
        max_load = simulation_state.generators(1).power_available;
        if power < eps
            avail_generator_power = 0;
        elseif power < min_load
            avail_generator_power = min_load;
        elseif power < max_load
            avail_generator_power = power;
        else
            avail_generator_power = max_load;
        end
    end
end

% dispatch_batteries
% dispatch batteries to deliver AC power
%
% queries the system's ability to provide AC power from batteries 
% without changing the charge state.
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% load : AC power requested from batteries
% inv_efficiency: inverter efficiency
%
% outputs
% -------
% avail_battery_power : AC battery power available for dispatch
function avail_battery_power = dispatch_batteries(simulation_state, simulation_parameters, load, inv_efficiency)
    avail_battery_power = 0;
    if (simulation_parameters.has_battery == true && simulation_parameters.has_converter == true) 
        avail_inverter_ac_capacity = simulation_parameters.converters(1).inverter_capacity - simulation_state.converters(1).inverter_power_output;
        avail_battery_dc_capacity = simulation_state.batteries(1).max_discharge_power + simulation_state.batteries(1).power_setpoint;
        avail_battery_ac_capacity = avail_battery_dc_capacity * (inv_efficiency/100);
        battery_max_discharge_power = min(avail_inverter_ac_capacity, avail_battery_ac_capacity);
        if battery_max_discharge_power >= load
            avail_battery_power = load;
        else
            avail_battery_power = battery_max_discharge_power;
        end
    end
end

% dispatch_wind
% dispatch wind to deliver AC power
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
%
% outputs
% -------
% avail_wind_power : AC wind power available for dispatch
% simulation_state : object holding current state of simulation
function [avail_wind_power, simulation_state] = dispatch_wind(simulation_state, simulation_parameters)
    if simulation_parameters.has_wind_turbine == true
        simulation_state.wind_turbines(1).power_setpoint = simulation_state.wind_turbines(1).power_available;
    else
        simulation_state.wind_turbines(1).power_setpoint = 0;
    end
    avail_wind_power = simulation_state.wind_turbines(1).power_setpoint;
end

% dispatch_solar
% dispatch solar to deliver AC power
% 
% inputs
% ------
% simulation_state : object holding current state of simulation
% simulation_parameters : object holding current parameters of simulation
% inv_efficiency : the DC->AC efficiency of the rectifier
%
% outputs
% -------
% avail_solar_ac_local : AC solar power available for dispatch
% simulation_state_local : object holding current state of simulation
function [avail_solar_ac_local, simulation_state_local] = dispatch_solar(simulation_state_local, simulation_parameters, inv_efficiency)
    if simulation_parameters.has_pv == true
        simulation_state_local.pvs(1).power_setpoint = simulation_state_local.pvs(1).power_available;
    else
        simulation_state_local.pvs(1).power_setpoint = 0;
    end
    avail_solar_dc = simulation_state_local.pvs(1).power_setpoint;
    avail_solar_ac_local = 0;
    if simulation_parameters.has_converter == true
        avail_solar_ac_local = min(simulation_parameters.converters(1).inverter_capacity, avail_solar_dc * inv_efficiency/100);
    end
end

end
