% Define parameters
c = 3e8;        % propagation speed
fc = 26e9;      % carrier frequency
lambda = c/fc;  % wavelength
AntArray = [8 8];

% Create antenna array
txarray = phased.URA('Size',AntArray,'ElementSpacing',[lambda/2 lambda/2]);
txarray.Element.BackBaffled = true;

%Read angles from CSV file
angles = csvread('angles.csv'); % Assuming angles.csv contains two columns for startA and startE

% Create figure
hFig = figure(1);
set(hFig,'Position',[300 100 800 600]);
set(gcf,'color','w');

% Loop through each set of angles
for idx = 1:size(angles, 1)
    startA = angles(idx, 1);
    startE = angles(idx, 2);
    % Compute steering vector
    steer_ang = [startA	; startE];
    stv = phased.SteeringVector('SensorArray',txarray);
    w = stv(fc,steer_ang);

    % Plot antenna array pattern
    subplot(2,2,1);
    pattern(txarray,fc,[0:360],[-90:90],'PropagationSpeed',c,'CoordinateSystem','polar','Type','powerdb', 'Weights',w)
    view(90,20);
    sTitle = sprintf("Antenna Array = %d by %d\nSteering Angle=[%d %d]",AntArray(1),AntArray(2),steer_ang(1),steer_ang(2));
    title(sTitle,'FontWeight','normal');
    set(gca,'fontsize',8);

    % Plot steering weights
    subplot(2,2,2);
    hold on;
    for y = 1:8
        for x = 1:8
            circle(x,y,0.5);
            vx1 = x;
            vy1 = y;
            vx2 = x + 0.5*real(w(8*(x-1)+y));
            vy2 = y + 0.5*imag(w(8*(x-1)+y));
            line([vx1 vx2],[vy1 vy2],'Color','red');
            plot(vx2,vy2,'ro','MarkerFaceColor',[1 0 0],'MarkerSize',3);
        end
    end  
    axis([0 9 0 9]);
    title('Steering Weight','FontWeight','normal');
    set(gca,'xticklabel',[]);set(gca,'yticklabel',[]);
    set(gca,'xtick',[]);set(gca,'ytick',[]);
    set(gca,'fontsize',8);
    daspect([1 1 1]);
    box on;
    hold off;

    % Plot directivity in elevation
    subplot(2,2,3);
    [pat,~,elevation] = pattern(txarray,fc,0,[-90:90],'PropagationSpeed',c,'CoordinateSystem','polar','Type','directivity', 'Weights',w);
    pat(isinf(pat)|isnan(pat)) = -40;
    pat(pat<=-40) = -40;
    polarplot(elevation*pi/180,pat,'r-');
    rlim([-40 30]);
    rticks([-40 -20 0 10 20 30]);
    title('Directivity(dBi)[Elevation @ Azimuth = 0]','FontWeight','normal');
    set(gca,'fontsize',8);

    % Plot directivity in azimuth
    subplot(2,2,4);
    [pat,azimuth,~] = pattern(txarray,fc,[0:360],0,'PropagationSpeed',c,'CoordinateSystem','polar','Type','directivity', 'Weights',w);
    pat(isinf(pat)|isnan(pat)) = -40;
    pat(pat<=-40) = -40;
    polarplot(azimuth*pi/180,pat,'r-');
    rlim([-40 30]);
    rticks([-40 -20 0 10 20 30]);
    title('Directivity(dBi)[Azimuth @ Elevation = 0]','FontWeight','normal');
    set(gca,'fontsize',8);

    % Pause for a short time to see the pattern
    pause(0.1);
end

% Function to draw circles
function circle(x,y,r)
    t = linspace(0,2*pi,20);
    plot(x+r*cos(t),y+r*sin(t),'k-');
end
