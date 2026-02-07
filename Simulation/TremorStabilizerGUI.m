function TremorStabilizerGUI()
    % Create larger figure for all components
    fig = uifigure('Name', 'Advanced Hand Tremor Stabilization Control', ...
                   'Position', [50, 50, 1600, 900]);
    
    % Main grid layout
    mainGrid = uigridlayout(fig, [3, 3]);
    mainGrid.RowHeight = {'1x', '1x', 60};
    mainGrid.ColumnWidth = {'1x', '1x', '1x'};
    
    % ====================================================================
    % TOP LEFT: 3D VISUALIZATION OF SIMPLE SPOON
    % ====================================================================
    panel3D = uipanel(mainGrid, 'Title', '🥄 3D Real-Time Spoon Stabilization', ...
                      'FontSize', 13, 'FontWeight', 'bold');
    panel3D.Layout.Row = 1;
    panel3D.Layout.Column = 1;
    
    ax3D = uiaxes(panel3D, 'Position', [10, 10, 450, 350]);
    view(ax3D, 3);
    axis(ax3D, [-2 2 -1 1 -0.5 3.5]);
    grid(ax3D, 'on');
    xlabel(ax3D, 'X'); ylabel(ax3D, 'Y'); zlabel(ax3D, 'Z');
    title(ax3D, 'Spoon Motion (Rotatable - Use Mouse)');
    hold(ax3D, 'on');
    ax3D.Toolbar.Visible = 'on';  % Enable rotation
    
    % Create simple paddle-like spoon shapes
    [X_spoon, Y_spoon, Z_spoon] = createSimpleSpoon();
    
    % Input (unstabilized) spoon - RED with transparency
    h_input = surf(ax3D, X_spoon, Y_spoon, Z_spoon + 2, ...
                   'FaceColor', [1, 0.2, 0.2], 'FaceAlpha', 0.7, ...
                   'EdgeColor', [0.5, 0, 0], 'LineWidth', 1, ...
                   'DisplayName', 'Input (Unstabilized)');
    
    % Stabilized spoon - BLUE solid
    h_stable = surf(ax3D, X_spoon, Y_spoon, Z_spoon + 0.5, ...
                    'FaceColor', [0.2, 0.5, 1], 'FaceAlpha', 0.95, ...
                    'EdgeColor', [0, 0, 0.5], 'LineWidth', 1.5, ...
                    'DisplayName', 'Stabilized Output');
    
    % Show servo mounting point (gimbal at handle base)
    plot3(ax3D, -1.2, 0, 2.5, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', ...
          'DisplayName', 'Servo Gimbal Mount');
    plot3(ax3D, -1.2, 0, 1, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
    
    % Add axis indicators at servo location
    quiver3(ax3D, -1.2, 0, 2.5, 0.3, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, ...
            'DisplayName', 'X-Servo Axis');
    quiver3(ax3D, -1.2, 0, 2.5, 0, 0.3, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5, ...
            'DisplayName', 'Y-Servo Axis');
    
    % Add text annotations
    text(ax3D, -1.5, 0.3, 2.5, 'Servos', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');
    text(ax3D, 0.8, 0, 2.7, 'Bowl', 'FontSize', 9, 'Color', [1, 0, 0]);
    text(ax3D, 0.8, 0, 1.2, 'Bowl', 'FontSize', 9, 'Color', [0, 0, 1]);
    text(ax3D, -1.5, 0, 2.0, 'Handle', 'FontSize', 9, 'Color', [0.5, 0, 0]);
    text(ax3D, -1.5, 0, 0.5, 'Handle', 'FontSize', 9, 'Color', [0, 0, 0.5]);
    
    % Add legend
    legend(ax3D, 'Location', 'northwest', 'FontSize', 8);
    
    lighting(ax3D, 'gouraud');
    lightangle(ax3D, 45, 30);
    
    % ====================================================================
    % TOP CENTER: PID PARAMETER TUNING
    % ====================================================================
    panelPID = uipanel(mainGrid, 'Title', '⚙️ PID Controller Tuning', ...
                       'FontSize', 13, 'FontWeight', 'bold');
    panelPID.Layout.Row = 1;
    panelPID.Layout.Column = 2;
    
    % Auto-tune button
    autoTuneBtn = uibutton(panelPID, 'Text', '🤖 Auto-Tune PID (Ziegler-Nichols)', ...
                           'Position', [20, 320, 250, 35], 'FontSize', 12, ...
                           'BackgroundColor', [0.8, 0.9, 1]);
    
    tuneStatusLabel = uilabel(panelPID, 'Text', 'Status: Manual Mode', ...
                              'Position', [280, 325, 200, 25], 'FontSize', 11);
    
    % Manual PID Tuners - X-axis
    uilabel(panelPID, 'Text', '━━━ X-Axis PID Gains ━━━', ...
            'Position', [20, 285, 450, 20], 'FontSize', 12, 'FontWeight', 'bold');
    
    uilabel(panelPID, 'Text', 'Kp_x:', 'Position', [20, 255, 50, 20]);
    kpx_slider = uislider(panelPID, 'Position', [80, 265, 200, 3], ...
                          'Limits', [0, 5], 'Value', 1.5);
    kpx_label = uilabel(panelPID, 'Text', '1.50', ...
                        'Position', [290, 255, 50, 20], 'FontWeight', 'bold');
    
    uilabel(panelPID, 'Text', 'Ki_x:', 'Position', [20, 225, 50, 20]);
    kix_slider = uislider(panelPID, 'Position', [80, 235, 200, 3], ...
                          'Limits', [0, 0.5], 'Value', 0.05);
    kix_label = uilabel(panelPID, 'Text', '0.05', ...
                        'Position', [290, 225, 50, 20], 'FontWeight', 'bold');
    
    uilabel(panelPID, 'Text', 'Kd_x:', 'Position', [20, 195, 50, 20]);
    kdx_slider = uislider(panelPID, 'Position', [80, 205, 200, 3], ...
                          'Limits', [0, 3], 'Value', 1.2);
    kdx_label = uilabel(panelPID, 'Text', '1.20', ...
                        'Position', [290, 195, 50, 20], 'FontWeight', 'bold');
    
    % Manual PID Tuners - Y-axis
    uilabel(panelPID, 'Text', '━━━ Y-Axis PID Gains ━━━', ...
            'Position', [20, 155, 450, 20], 'FontSize', 12, 'FontWeight', 'bold');
    
    uilabel(panelPID, 'Text', 'Kp_y:', 'Position', [20, 125, 50, 20]);
    kpy_slider = uislider(panelPID, 'Position', [80, 135, 200, 3], ...
                          'Limits', [0, 5], 'Value', 1.5);
    kpy_label = uilabel(panelPID, 'Text', '1.50', ...
                        'Position', [290, 125, 50, 20], 'FontWeight', 'bold');
    
    uilabel(panelPID, 'Text', 'Ki_y:', 'Position', [20, 95, 50, 20]);
    kiy_slider = uislider(panelPID, 'Position', [80, 105, 200, 3], ...
                          'Limits', [0, 0.5], 'Value', 0.05);
    kiy_label = uilabel(panelPID, 'Text', '0.05', ...
                        'Position', [290, 95, 50, 20], 'FontWeight', 'bold');
    
    uilabel(panelPID, 'Text', 'Kd_y:', 'Position', [20, 65, 50, 20]);
    kdy_slider = uislider(panelPID, 'Position', [80, 75, 200, 3], ...
                          'Limits', [0, 3], 'Value', 1.2);
    kdy_label = uilabel(panelPID, 'Text', '1.20', ...
                        'Position', [290, 65, 50, 20], 'FontWeight', 'bold');
    
    % Servo speed limit warning
    servoWarning = uilabel(panelPID, 'Text', '⚠️ 9g Servo Safe: Kd < 2.0 | Max Speed: 500°/s', ...
                           'Position', [20, 25, 450, 25], 'FontSize', 10, ...
                           'FontColor', [0.8, 0.4, 0], 'FontWeight', 'bold');
    
    % ====================================================================
    % TOP RIGHT: PID COMPONENT BREAKDOWN
    % ====================================================================
    panelComponents = uipanel(mainGrid, 'Title', '📊 PID Components (X-axis)', ...
                              'FontSize', 13, 'FontWeight', 'bold');
    panelComponents.Layout.Row = 1;
    panelComponents.Layout.Column = 3;
    
    axComponents = uiaxes(panelComponents, 'Position', [10, 10, 480, 350]);
    axComponents.XLim = [0, 100];
    axComponents.YLim = [-50, 50];
    axComponents.YLabel.String = 'Contribution (°)';
    axComponents.XLabel.String = 'Time Steps';
    axComponents.XGrid = 'on';
    axComponents.YGrid = 'on';
    hold(axComponents, 'on');
    
    lineP = plot(axComponents, NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'P (Proportional)');
    lineI = plot(axComponents, NaN, NaN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'I (Integral)');
    lineD = plot(axComponents, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'D (Derivative)');
    lineTotal = plot(axComponents, NaN, NaN, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Total Output');
    legend(axComponents, 'Location', 'northwest');
    
    % ====================================================================
    % MIDDLE LEFT: INPUT VIBRATION & ERROR PLOT
    % ====================================================================
    panelInput = uipanel(mainGrid, 'Title', '📈 Input Vibration & Error Analysis', ...
                         'FontSize', 13, 'FontWeight', 'bold');
    panelInput.Layout.Row = 2;
    panelInput.Layout.Column = 1;
    
    axInput = uiaxes(panelInput, 'Position', [10, 10, 480, 350]);
    axInput.XLim = [0, 100];
    axInput.YLim = [-6, 6];
    axInput.YLabel.String = 'Vibration (g)';
    axInput.XLabel.String = 'Time Steps';
    axInput.XGrid = 'on';
    axInput.YGrid = 'on';
    hold(axInput, 'on');
    
    inputXLine = plot(axInput, NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName', 'X-Vibration');
    inputYLine = plot(axInput, NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Y-Vibration');
    errorXLine = plot(axInput, NaN, NaN, 'm--', 'LineWidth', 1.5, 'DisplayName', 'X-Error');
    legend(axInput, 'Location', 'northwest');
    
    % ====================================================================
    % MIDDLE CENTER: CONTROL SLIDERS & STATUS
    % ====================================================================
    panelControl = uipanel(mainGrid, 'Title', '🎮 Manual Control & System Status', ...
                           'FontSize', 13, 'FontWeight', 'bold');
    panelControl.Layout.Row = 2;
    panelControl.Layout.Column = 2;
    
    % Tremor type selector
    uilabel(panelControl, 'Text', 'Tremor Type:', ...
            'Position', [20, 320, 120, 25], 'FontSize', 12, 'FontWeight', 'bold');
    tremorTypeDropdown = uidropdown(panelControl, ...
                                    'Items', {'Manual Control', 'Parkinson''s (4-6Hz)', 'Essential Tremor (4-12Hz)', 'Mixed Realistic'}, ...
                                    'Position', [140, 320, 200, 25], ...
                                    'Value', 'Manual Control');
    
    % Manual sliders
    uilabel(panelControl, 'Text', 'X-Axis Manual Input:', ...
            'Position', [20, 280, 200, 20], 'FontSize', 11);
    xSlider = uislider(panelControl, 'Position', [20, 270, 450, 3], ...
                       'Limits', [-5, 5], 'Value', 0, 'MajorTicks', -5:1:5);
    xSliderLabel = uilabel(panelControl, 'Text', '0.00 g', ...
                           'Position', [220, 245, 60, 20], 'FontWeight', 'bold');
    
    uilabel(panelControl, 'Text', 'Y-Axis Manual Input:', ...
            'Position', [20, 220, 200, 20], 'FontSize', 11);
    ySlider = uislider(panelControl, 'Position', [20, 210, 450, 3], ...
                       'Limits', [-5, 5], 'Value', 0, 'MajorTicks', -5:1:5);
    ySliderLabel = uilabel(panelControl, 'Text', '0.00 g', ...
                           'Position', [220, 185, 60, 20], 'FontWeight', 'bold');
    
    % Status panel
    statusPanel = uipanel(panelControl, 'Position', [20, 20, 450, 150], ...
                          'BackgroundColor', [0.95, 0.98, 1]);
    
    uilabel(statusPanel, 'Text', '📊 System Metrics', ...
            'Position', [10, 120, 430, 25], 'FontSize', 13, 'FontWeight', 'bold');
    
    rmsLabel = uilabel(statusPanel, 'Text', 'RMS Vibration: 0.00 g', ...
                       'Position', [10, 95, 200, 20], 'FontSize', 11);
    
    reductionLabel = uilabel(statusPanel, 'Text', 'Stabilization: 0.0%', ...
                             'Position', [220, 95, 200, 20], 'FontSize', 11);
    
    servoSpeedLabel = uilabel(statusPanel, 'Text', 'Servo Speed: 0 °/s', ...
                              'Position', [10, 70, 200, 20], 'FontSize', 11);
    
    maxDeflectionLabel = uilabel(statusPanel, 'Text', 'Max Deflection: 0.0°', ...
                                 'Position', [220, 70, 200, 20], 'FontSize', 11);
    
    tremorFreqLabel = uilabel(statusPanel, 'Text', 'Tremor Freq: - Hz', ...
                              'Position', [10, 45, 200, 20], 'FontSize', 11);
    
    statusLabel = uilabel(statusPanel, 'Text', 'Status: Ready', ...
                          'Position', [10, 15, 430, 25], 'FontSize', 12, ...
                          'FontWeight', 'bold', 'FontColor', [0, 0.6, 0]);
    
    % ====================================================================
    % MIDDLE RIGHT: OUTPUT SERVO & RESIDUAL TREMOR
    % ====================================================================
    panelOutput = uipanel(mainGrid, 'Title', '📉 Servo Output & Residual Tremor', ...
                          'FontSize', 13, 'FontWeight', 'bold');
    panelOutput.Layout.Row = 2;
    panelOutput.Layout.Column = 3;
    
    % Auto-scale checkbox
    autoScaleCheck = uicheckbox(panelOutput, 'Text', 'Auto-Scale Y-axis', ...
                                'Position', [350, 330, 130, 20], 'Value', false);
    
    axOutput = uiaxes(panelOutput, 'Position', [10, 10, 480, 310]);
    axOutput.XLim = [0, 100];
    axOutput.YLim = [-90, 90];
    axOutput.YLabel.String = 'Angle (°) / Residual (g)';
    axOutput.XLabel.String = 'Time Steps';
    axOutput.XGrid = 'on';
    axOutput.YGrid = 'on';
    hold(axOutput, 'on');
    
    outputXLine = plot(axOutput, NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName', 'X-Servo');
    outputYLine = plot(axOutput, NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Y-Servo');
    residualXLine = plot(axOutput, NaN, NaN, 'm--', 'LineWidth', 1.5, 'DisplayName', 'X-Residual (×10)');
    residualYLine = plot(axOutput, NaN, NaN, 'c--', 'LineWidth', 1.5, 'DisplayName', 'Y-Residual (×10)');
    legend(axOutput, 'Location', 'northwest');
    
    % ====================================================================
    % BOTTOM: CONTROL BUTTONS
    % ====================================================================
    buttonPanel = uipanel(mainGrid);
    buttonPanel.Layout.Row = 3;
    buttonPanel.Layout.Column = [1, 3];
    
    startBtn = uibutton(buttonPanel, 'Text', '▶️ Start Simulation', ...
                        'Position', [50, 10, 180, 40], 'FontSize', 13, ...
                        'BackgroundColor', [0.7, 1, 0.7], 'FontWeight', 'bold');
    
    stopBtn = uibutton(buttonPanel, 'Text', '⏸️ Stop', ...
                       'Position', [250, 10, 120, 40], 'FontSize', 13, ...
                       'BackgroundColor', [1, 1, 0.7], 'FontWeight', 'bold');
    
    resetBtn = uibutton(buttonPanel, 'Text', '🔄 Reset', ...
                        'Position', [390, 10, 120, 40], 'FontSize', 13, ...
                        'BackgroundColor', [1, 0.8, 0.8], 'FontWeight', 'bold');
    
    saveBtn = uibutton(buttonPanel, 'Text', '💾 Save Data', ...
                       'Position', [530, 10, 140, 40], 'FontSize', 13, ...
                       'BackgroundColor', [0.9, 0.9, 1], 'FontWeight', 'bold');
    
    exportBtn = uibutton(buttonPanel, 'Text', '📸 Export 3D View', ...
                         'Position', [690, 10, 160, 40], 'FontSize', 13, ...
                         'BackgroundColor', [1, 0.9, 0.8], 'FontWeight', 'bold');
    
    servoInfoBtn = uibutton(buttonPanel, 'Text', 'ℹ️ Servo Info', ...
                            'Position', [870, 10, 130, 40], 'FontSize', 13, ...
                            'BackgroundColor', [0.9, 1, 0.9], 'FontWeight', 'bold');
    
    % ====================================================================
    % INITIALIZE SYSTEM
    % ====================================================================
    stabilizer = TremorStabilizer();
    stabilizer.Kd_x = 1.2;
    stabilizer.Kd_y = 1.2;
    
    % Data storage
    maxPoints = 100;
    timeSteps = 1:maxPoints;
    xVibHistory = zeros(1, maxPoints);
    yVibHistory = zeros(1, maxPoints);
    xServoHistory = zeros(1, maxPoints);
    yServoHistory = zeros(1, maxPoints);
    xErrorHistory = zeros(1, maxPoints);
    yErrorHistory = zeros(1, maxPoints);
    xResidualHistory = zeros(1, maxPoints);
    yResidualHistory = zeros(1, maxPoints);
    
    pHistory = zeros(1, maxPoints);
    iHistory = zeros(1, maxPoints);
    dHistory = zeros(1, maxPoints);
    
    currentStep = 0;
    currentTime = 0;
    dt = 1/stabilizer.sampling_freq;
    isRunning = false;
    
    % ====================================================================
    % PID SLIDER CALLBACKS
    % ====================================================================
    kpx_slider.ValueChangingFcn = @(~, event) updatePIDLabel(kpx_label, event.Value, 'Kp_x', stabilizer);
    kix_slider.ValueChangingFcn = @(~, event) updatePIDLabel(kix_label, event.Value, 'Ki_x', stabilizer);
    kdx_slider.ValueChangingFcn = @(~, event) updatePIDLabel(kdx_label, event.Value, 'Kd_x', stabilizer);
    kpy_slider.ValueChangingFcn = @(~, event) updatePIDLabel(kpy_label, event.Value, 'Kp_y', stabilizer);
    kiy_slider.ValueChangingFcn = @(~, event) updatePIDLabel(kiy_label, event.Value, 'Ki_y', stabilizer);
    kdy_slider.ValueChangingFcn = @(~, event) updatePIDLabel(kdy_label, event.Value, 'Kd_y', stabilizer);
    
    function updatePIDLabel(label, value, param, stab)
        label.Text = sprintf('%.2f', value);
        stab.(param) = value;
        
        if contains(param, 'Kd') && value > 2.0
            servoWarning.FontColor = [1, 0, 0];
            servoWarning.Text = '⚠️ WARNING: Kd too high! May damage servo!';
        else
            servoWarning.FontColor = [0.8, 0.4, 0];
            servoWarning.Text = '⚠️ 9g Servo Safe: Kd < 2.0 | Max Speed: 500°/s';
        end
    end
    
    % ====================================================================
    % AUTO-TUNE CALLBACK
    % ====================================================================
    autoTuneBtn.ButtonPushedFcn = @(~, ~) autoTunePID();
    
    function autoTunePID()
        tuneStatusLabel.Text = 'Status: Auto-tuning...';
        tuneStatusLabel.FontColor = [1, 0.5, 0];
        drawnow;
        
        rms_tremor = sqrt(mean(xVibHistory.^2 + yVibHistory.^2));
        
        if rms_tremor < 0.1
            tuneStatusLabel.Text = 'Error: No tremor detected';
            tuneStatusLabel.FontColor = [1, 0, 0];
            return;
        end
        
        Ku = 2.5 * (1 / rms_tremor);
        Tu = 0.2;
        
        Kp_new = 0.6 * Ku;
        Ki_new = 1.2 * Ku / Tu;
        Kd_new = 0.075 * Ku * Tu;
        
        Kp_new = min(max(Kp_new, 0.5), 5.0);
        Ki_new = min(max(Ki_new, 0.01), 0.5);
        Kd_new = min(max(Kd_new, 0.5), 1.8);
        
        kpx_slider.Value = Kp_new; kpy_slider.Value = Kp_new;
        kix_slider.Value = Ki_new; kiy_slider.Value = Ki_new;
        kdx_slider.Value = Kd_new; kdy_slider.Value = Kd_new;
        
        stabilizer.Kp_x = Kp_new; stabilizer.Kp_y = Kp_new;
        stabilizer.Ki_x = Ki_new; stabilizer.Ki_y = Ki_new;
        stabilizer.Kd_x = Kd_new; stabilizer.Kd_y = Kd_new;
        
        kpx_label.Text = sprintf('%.2f', Kp_new); kpy_label.Text = sprintf('%.2f', Kp_new);
        kix_label.Text = sprintf('%.2f', Ki_new); kiy_label.Text = sprintf('%.2f', Ki_new);
        kdx_label.Text = sprintf('%.2f', Kd_new); kdy_label.Text = sprintf('%.2f', Kd_new);
        
        tuneStatusLabel.Text = 'Status: Auto-tuned!';
        tuneStatusLabel.FontColor = [0, 0.6, 0];
        
        fprintf('✓ Auto-tune: Kp=%.2f, Ki=%.3f, Kd=%.2f\n', Kp_new, Ki_new, Kd_new);
    end
    
    % ====================================================================
    % REALISTIC TREMOR GENERATOR
    % ====================================================================
    function [xVib, yVib] = generateRealisticTremor(t, tremorType)
        switch tremorType
            case 'Parkinson''s (4-6Hz)'
                f1 = 4.7; f2 = 6.2; f3 = 11.7;
                amp1 = 1.5; amp2 = 0.8; amp3 = 0.3;
                
            case 'Essential Tremor (4-12Hz)'
                f1 = 5.5; f2 = 7.8; f3 = 10.9;
                amp1 = 2.0; amp2 = 1.2; amp3 = 0.6;
                
            case 'Mixed Realistic'
                f1 = 4.5 + 0.5*rand();
                f2 = 7.0 + rand();
                f3 = 11.0 + rand();
                amp1 = 1.5 + 0.5*randn();
                amp2 = 0.8 + 0.3*randn();
                amp3 = 0.4 + 0.2*randn();
                
            otherwise
                xVib = 0; yVib = 0;
                return;
        end
        
        amplitude_envelope = 0.7 + 0.3*sin(0.3*t) + 0.2*rand();
        burst_factor = (mod(floor(t), 8) < 6) * 1.0 + (mod(floor(t), 8) >= 6) * 0.2;
        
        base_tremor = amp1*sin(2*pi*f1*t) + amp2*sin(2*pi*f2*t + pi/3) + amp3*sin(2*pi*f3*t + pi/6);
        noise = 0.3*randn();
        
        xVib = base_tremor * amplitude_envelope * burst_factor + noise;
        yVib = base_tremor * amplitude_envelope * burst_factor * cos(pi/4) + 0.3*randn();
        
        xVib = max(-5, min(5, xVib));
        yVib = max(-5, min(5, yVib));
    end
    
    % ====================================================================
    % SLIDER CALLBACKS
    % ====================================================================
    xSlider.ValueChangingFcn = @(~, event) updateManualSlider(event.Value, 'x');
    ySlider.ValueChangingFcn = @(~, event) updateManualSlider(event.Value, 'y');
    
    function updateManualSlider(value, axis)
        if strcmp(axis, 'x')
            xSliderLabel.Text = sprintf('%.2f g', value);
        else
            ySliderLabel.Text = sprintf('%.2f g', value);
        end
    end
    
    % ====================================================================
    % MAIN UPDATE FUNCTION
    % ====================================================================
    function updateSystem(xVib, yVib)
        currentTime = currentTime + dt;
        currentStep = currentStep + 1;
        
        [servoX, servoY, pComp, iComp, dComp] = computeServoWithComponents(xVib, yVib);
        
        errorX = -xVib;
        residualX = xVib + servoX/18;
        residualY = yVib + servoY/18;
        
        if currentStep > maxPoints
            xVibHistory = [xVibHistory(2:end), xVib];
            yVibHistory = [yVibHistory(2:end), yVib];
            xServoHistory = [xServoHistory(2:end), servoX];
            yServoHistory = [yServoHistory(2:end), servoY];
            xErrorHistory = [xErrorHistory(2:end), errorX];
            xResidualHistory = [xResidualHistory(2:end), residualX];
            yResidualHistory = [yResidualHistory(2:end), residualY];
            pHistory = [pHistory(2:end), pComp];
            iHistory = [iHistory(2:end), iComp];
            dHistory = [dHistory(2:end), dComp];
        else
            xVibHistory(currentStep) = xVib;
            yVibHistory(currentStep) = yVib;
            xServoHistory(currentStep) = servoX;
            yServoHistory(currentStep) = servoY;
            xErrorHistory(currentStep) = errorX;
            xResidualHistory(currentStep) = residualX;
            yResidualHistory(currentStep) = residualY;
            pHistory(currentStep) = pComp;
            iHistory(currentStep) = iComp;
            dHistory(currentStep) = dComp;
        end
        
        update3DSpoon(xVib, yVib, servoX, servoY);
        updateAllPlots();
        updateMetrics();
        
        drawnow;
    end
    
    function [servoX, servoY, pComp, iComp, dComp] = computeServoWithComponents(xVib, yVib)
        [servoX, servoY] = stabilizer.computeServoPositions(xVib, yVib, currentTime);
        
        errorX = -xVib;
        pComp = stabilizer.Kp_x * errorX;
        iComp = stabilizer.Ki_x * stabilizer.integral_x;
        dComp = stabilizer.Kd_x * (errorX - stabilizer.prev_error_x) / dt;
    end
    
    function update3DSpoon(xVib, yVib, servoX, servoY)
        % Input spoon rotations (exaggerated)
        angleX_input = xVib * 8;
        angleY_input = yVib * 8;
        
        % Stabilized spoon rotations
        angleX_stable = (xVib * 8 + servoX * 0.3);
        angleY_stable = (yVib * 8 + servoY * 0.3);
        
        % Rotate input spoon (RED)
        [X_in, Y_in, Z_in] = rotateSimpleSpoon(X_spoon, Y_spoon, Z_spoon, angleX_input, angleY_input);
        h_input.XData = X_in;
        h_input.YData = Y_in;
        h_input.ZData = Z_in + 2;
        
        % Rotate stabilized spoon (BLUE)
        [X_st, Y_st, Z_st] = rotateSimpleSpoon(X_spoon, Y_spoon, Z_spoon, angleX_stable, angleY_stable);
        h_stable.XData = X_st;
        h_stable.YData = Y_st;
        h_stable.ZData = Z_st + 0.5;
    end
    
    function updateAllPlots()
        if currentStep <= maxPoints
            steps = 1:currentStep;
        else
            steps = timeSteps;
        end
        
        inputXLine.XData = steps; inputXLine.YData = xVibHistory(steps);
        inputYLine.XData = steps; inputYLine.YData = yVibHistory(steps);
        errorXLine.XData = steps; errorXLine.YData = xErrorHistory(steps);
        
        outputXLine.XData = steps; outputXLine.YData = xServoHistory(steps);
        outputYLine.XData = steps; outputYLine.YData = yServoHistory(steps);
        residualXLine.XData = steps; residualXLine.YData = xResidualHistory(steps);
        residualYLine.XData = steps; residualYLine.YData = yResidualHistory(steps);
        
        if autoScaleCheck.Value
            maxVal = max(abs([xServoHistory, yServoHistory]));
            axOutput.YLim = [-maxVal*1.2, maxVal*1.2];
        else
            axOutput.YLim = [-90, 90];X-Error
        end
        
        lineP.XData = steps; lineP.YData = pHistory(steps);
        lineI.XData = steps; lineI.YData = iHistory(steps);
        lineD.XData = steps; lineD.YData = dHistory(steps);
        lineTotal.XData = steps; lineTotal.YData = pHistory(steps) + iHistory(steps) + dHistory(steps);
    end
    
    function updateMetrics()
        rms_vib = sqrt(mean(xVibHistory.^2 + yVibHistory.^2));
        rmsLabel.Text = sprintf('RMS Vibration: %.2f g', rms_vib);
        
        rms_residual = sqrt(mean(xResidualHistory.^2 + yResidualHistory.^2));
        if rms_vib > 0.1
            reduction = (1 - rms_residual/rms_vib) * 100;
            reductionLabel.Text = sprintf('Stabilization: %.1f%%', max(0, reduction));
        end
        
        if currentStep > 1
            servoSpeed = abs(xServoHistory(end) - xServoHistory(max(1, end-1))) / dt;
            servoSpeedLabel.Text = sprintf('Servo Speed: %.0f °/s', servoSpeed);
            servoSpeedLabel.FontColor = (servoSpeed > 500) * [1,0,0] + (servoSpeed <= 500) * [0,0,0];
        end
        
        maxDefl = max(abs([xServoHistory, yServoHistory]));
        maxDeflectionLabel.Text = sprintf('Max Deflection: %.1f°', maxDefl);
    end
    
    % ====================================================================
    % BUTTON CALLBACKS
    % ====================================================================
    startBtn.ButtonPushedFcn = @(~, ~) startSimulation();
    stopBtn.ButtonPushedFcn = @(~, ~) stopSimulation();
    resetBtn.ButtonPushedFcn = @(~, ~) resetSystem();
    saveBtn.ButtonPushedFcn = @(~, ~) saveData();
    exportBtn.ButtonPushedFcn = @(~, ~) export3DView();
    servoInfoBtn.ButtonPushedFcn = @(~, ~) showServoInfo();
    
    function startSimulation()
        isRunning = true;
        statusLabel.Text = 'Status: Running...';
        statusLabel.FontColor = [0, 0.6, 0];
        
        tremorType = tremorTypeDropdown.Value;
        
        if strcmp(tremorType, 'Manual Control')
            while isRunning
                xVib = xSlider.Value;
                yVib = ySlider.Value;
                updateSystem(xVib, yVib);
                pause(dt);
            end
        else
            duration = 30;
            steps = duration / dt;
            
            for i = 1:steps
                if ~isRunning, break; end
                
                t = currentTime + dt;
                [xVib, yVib] = generateRealisticTremor(t, tremorType);
                
                xSlider.Value = xVib;
                ySlider.Value = yVib;
                xSliderLabel.Text = sprintf('%.2f g', xVib);
                ySliderLabel.Text = sprintf('%.2f g', yVib);
                
                updateSystem(xVib, yVib);
                pause(dt);
            end
        end
        
        statusLabel.Text = 'Status: Complete';
        isRunning = false;
    end
    
    function stopSimulation()
        isRunning = false;
        statusLabel.Text = 'Status: Stopped';
        statusLabel.FontColor = [0.8, 0.4, 0];
    end
    
    function resetSystem()
        isRunning = false;
        currentStep = 0;
        currentTime = 0;
        
        xSlider.Value = 0; ySlider.Value = 0;
        xSliderLabel.Text = '0.00 g'; ySliderLabel.Text = '0.00 g';
        
        stabilizer.resetPID();
        
        xVibHistory(:) = 0; yVibHistory(:) = 0;
        xServoHistory(:) = 0; yServoHistory(:) = 0;
        xErrorHistory(:) = 0; xResidualHistory(:) = 0; yResidualHistory(:) = 0;
        pHistory(:) = 0; iHistory(:) = 0; dHistory(:) = 0;
        
        inputXLine.XData = NaN; inputXLine.YData = NaN;
        inputYLine.XData = NaN; inputYLine.YData = NaN;
        errorXLine.XData = NaN; errorXLine.YData = NaN;
        outputXLine.XData = NaN; outputXLine.YData = NaN;
        outputYLine.XData = NaN; outputYLine.YData = NaN;
        residualXLine.XData = NaN; residualXLine.YData = NaN;
        residualYLine.XData = NaN; residualYLine.YData = NaN;
        lineP.XData = NaN; lineP.YData = NaN;
        lineI.XData = NaN; lineI.YData = NaN;
        lineD.XData = NaN; lineD.YData = NaN;
        lineTotal.XData = NaN; lineTotal.YData = NaN;
        
        statusLabel.Text = 'Status: Reset Complete';
        statusLabel.FontColor = [0, 0, 0];
        
        fprintf('✓ System Reset\n');
    end
    
    function saveData()
        data = table(xVibHistory', yVibHistory', xServoHistory', yServoHistory', ...
                     xResidualHistory', yResidualHistory', ...
                     'VariableNames', {'X_Vib', 'Y_Vib', 'X_Servo', 'Y_Servo', 'X_Residual', 'Y_Residual'});
        filename = sprintf('tremor_data_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'));
        writetable(data, filename);
        fprintf('💾 Data saved: %s\n', filename);
        statusLabel.Text = sprintf('Saved: %s', filename);
    end
    
    function export3DView()
        exportgraphics(ax3D, sprintf('3d_spoon_%s.png', datestr(now, 'yyyymmdd_HHMMSS')));
        fprintf('📸 3D view exported\n');
    end
    
    function showServoInfo()
        msg = sprintf(['SERVO MOUNTING CONFIGURATION:\n\n' ...
                      '📍 Location: Handle Base (Gimbal Mount)\n\n' ...
                      'X-Axis Servo (Roll Control):\n' ...
                      '  • Controls side-to-side tilt\n' ...
                      '  • Mounted at green marker position\n' ...
                      '  • Range: -90° to +90°\n' ...
                      '  • Red arrow shows rotation axis\n\n' ...
                      'Y-Axis Servo (Pitch Control):\n' ...
                      '  • Controls up-down tilt\n' ...
                      '  • Mounted perpendicular to X-servo\n' ...
                      '  • Range: -90° to +90°\n' ...
                      '  • Blue arrow shows rotation axis\n\n' ...
                      '🔧 Servo Specs (TowerPro SG90):\n' ...
                      '  • Speed: 0.1s/60° (≈500°/s max)\n' ...
                      '  • Safe Kd: < 2.0\n' ...
                      '  • Operating freq: 50Hz PWM\n\n' ...
                      '✅ This gimbal configuration stabilizes\n' ...
                      '   the bowl end while you hold the handle.']);
        uialert(fig, msg, 'Servo Mounting Information', 'Icon', 'info');
    end
end
% ========================================================================
% HELPER FUNCTIONS FOR SIMPLE PADDLE-LIKE SPOON
% ========================================================================
function [X, Y, Z] = createSimpleSpoon()
    % Create simple flat paddle-like spoon shape
    % More robust implementation
    
    n_points = 40;
    
    % Define spoon outline points along x-axis
    x_vals = linspace(-1.5, 1.0, n_points);
    
    % Calculate width at each point
    widths = zeros(size(x_vals));
    
    for i = 1:length(x_vals)
        x = x_vals(i);
        
        if x < -0.2  % Handle region (narrow)
            widths(i) = 0.06;
            
        elseif x < 0.7  % Expanding bowl region
            % Linear expansion from 0.06 to 0.25
            progress = (x + 0.2) / 0.9;  % 0 to 1
            widths(i) = 0.06 + progress * 0.19;
            
        else  % Rounded tip region
            % Smooth rounded end using cosine
            progress = (x - 0.7) / 0.3;  % 0 to 1
            widths(i) = 0.25 * cos(progress * pi/2);
        end
    end
    
    % Create mesh grid for spoon surface
    X = zeros(2, n_points);
    Y = zeros(2, n_points);
    Z = zeros(2, n_points);
    
    for i = 1:n_points
        X(:, i) = [x_vals(i); x_vals(i)];
        Y(:, i) = [-widths(i); widths(i)];
        Z(:, i) = [0; 0];
    end
end

function [X_rot, Y_rot, Z_rot] = rotateSimpleSpoon(X, Y, Z, angleX, angleY)
    % Rotate spoon around gimbal point at x=-1.2
    gimbal_x = -1.2;
    gimbal_z = 0;
    
    % Translate to gimbal origin
    X = X - gimbal_x;
    Z = Z - gimbal_z;
    
    angleX_rad = deg2rad(angleX);
    angleY_rad = deg2rad(angleY);
    
    % Rotation matrices
    Rx = [1, 0, 0; 0, cos(angleX_rad), -sin(angleX_rad); 0, sin(angleX_rad), cos(angleX_rad)];
    Ry = [cos(angleY_rad), 0, sin(angleY_rad); 0, 1, 0; -sin(angleY_rad), 0, cos(angleY_rad)];
    R = Ry * Rx;
    
    % Apply rotation
    points = [X(:), Y(:), Z(:)]';
    rotated = R * points;
    
    X_rot = reshape(rotated(1,:), size(X)) + gimbal_x;
    Y_rot = reshape(rotated(2,:), size(Y));
    Z_rot = reshape(rotated(3,:), size(Z)) + gimbal_z;
end
