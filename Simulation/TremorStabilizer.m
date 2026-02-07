% ========================================================================
% TREMOR STABILIZER CLASS
% Save this as: TremorStabilizer.m
% ========================================================================
classdef TremorStabilizer < handle
    properties
        Kp_x = 1.5; Ki_x = 0.05; Kd_x = 0.8;
        Kp_y = 1.5; Ki_y = 0.05; Kd_y = 0.8;
        servo_min = -90; servo_max = 90; servo_center = 0;
        accel_sensitivity = 8192; g = 9.81;
        filter_order = 2; cutoff_freq = 10; sampling_freq = 100;
        integral_x = 0; integral_y = 0;
        prev_error_x = 0; prev_error_y = 0; prev_time = 0;
        filter_state_x; filter_state_y; lpf_num; lpf_den;
    end
    
    methods
        function obj = TremorStabilizer()
            [obj.lpf_num, obj.lpf_den] = butter(obj.filter_order, ...
                obj.cutoff_freq / (obj.sampling_freq/2), 'low');
            obj.filter_state_x = zeros(max(length(obj.lpf_num), ...
                length(obj.lpf_den)) - 1, 1);
            obj.filter_state_y = zeros(max(length(obj.lpf_num), ...
                length(obj.lpf_den)) - 1, 1);
            obj.prev_time = 0;
        end
        
        function [servo_x, servo_y] = computeServoPositions(obj, ...
                vibration_x, vibration_y, current_time)
            [filtered_x, obj.filter_state_x] = filter(obj.lpf_num, ...
                obj.lpf_den, vibration_x, obj.filter_state_x);
            [filtered_y, obj.filter_state_y] = filter(obj.lpf_num, ...
                obj.lpf_den, vibration_y, obj.filter_state_y);
            
            if obj.prev_time == 0
                dt = 1 / obj.sampling_freq;
            else
                dt = current_time - obj.prev_time;
            end
            obj.prev_time = current_time;
            
            error_x = -filtered_x;
            obj.integral_x = obj.integral_x + error_x * dt;
            derivative_x = (error_x - obj.prev_error_x) / dt;
            control_signal_x = obj.Kp_x * error_x + obj.Ki_x * obj.integral_x + obj.Kd_x * derivative_x;
            obj.prev_error_x = error_x;
            
            error_y = -filtered_y;
            obj.integral_y = obj.integral_y + error_y * dt;
            derivative_y = (error_y - obj.prev_error_y) / dt;
            control_signal_y = obj.Kp_y * error_y + obj.Ki_y * obj.integral_y + obj.Kd_y * derivative_y;
            obj.prev_error_y = error_y;
            
            servo_x = obj.mapToServoAngle(control_signal_x);
            servo_y = obj.mapToServoAngle(control_signal_y);
        end
        
        function servo_angle = mapToServoAngle(obj, control_signal)
            scaling_factor = 9.0;
            servo_angle = control_signal * scaling_factor;
            servo_angle = max(obj.servo_min, min(obj.servo_max, servo_angle));
        end
        
        function resetPID(obj)
            obj.integral_x = 0; obj.integral_y = 0;
            obj.prev_error_x = 0; obj.prev_error_y = 0;
            obj.prev_time = 0;
            obj.filter_state_x = zeros(size(obj.filter_state_x));
            obj.filter_state_y = zeros(size(obj.filter_state_y));
        end
    end
end
