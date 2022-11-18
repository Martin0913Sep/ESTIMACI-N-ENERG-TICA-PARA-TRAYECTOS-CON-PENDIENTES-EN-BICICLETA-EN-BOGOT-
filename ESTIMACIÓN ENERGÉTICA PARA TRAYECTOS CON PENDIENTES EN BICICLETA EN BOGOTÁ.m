%filename="Cementerio septima comparacion malo NO USAR.xlsx"; %Cementerio
%filename="Barrios unidos.xlsx"; %Barrios Unidos a Javeriana
%filename="Teusaquillo.xlsx"; %Teusaquillo a Javeriana
%filename="Candelaria.xlsx"; %Candelaria a Javeriana 
%filename="Engativa.xlsx"; %Engativá a Javeriana
%filename="Santa fe.xlsx"; %Santa fe a Javeriana REVISAR
%filename="SUBA.xlsx"; %SUBA a Javeriana
%filename="Usaquen.xlsx"; %Usaquen a Javeriana
%filename="Los martires.xlsx"; %Mártires a Javeriana REVISAR
%filename="Fontibon.xlsx"; %Fontibón a Javeriana 
%filename="Kennedy.xlsx"; %Kennedy a Javeriana
%filename="San cristobal.xlsx"; %San cristobal a Javeriana
%filename="Antonio Nario.xlsx"; %Antonio Nario a Javeriana
%filename="Tunjuelito.xlsx"; %Tunjuelito a Javeriana 
%filename="Bosa.xlsx"; %Bosa a Javeriana 
%filename="Rafael U.xlsx"; %Rafael U a Javeriana
%filename="Ciudad Bolivar.xlsx"; %Ciudad Bolivar a Javeriana
%filename="Puente aranda.xlsx"; %Puente aranda a Javeriana



ciclista = 70;%Cyclist Mass[Kg]
    bicicleta = 10;%Bike Mass[Kg]
    mt = ciclista + bicicleta;%Total Mass[Kg]
    deltat = 1; %Delta de tiempo [s]
    Fw = 0.0044;%Factor associated with wheel rotation [m^2]
    g = 9.81;%Gravity [m/s^2]
    Gr = 0;%road gradient [° Grados] Arranque en plano. 
    b0 = 0.091;%1st coefficient of wheel bearing [N]
    b1 = 0.0087;%2nd coefficient of wheel bearing [N*s/m]
    %Vg(1)= -0.3; %Valor inicial de velocidad [m/s]
    Vg(1)= 0; %Valor inicial de velocidad [m/s]
    dist(1) = 0; %Valor inicial de distancia [m]
    I = 0.14;%Inertia [kg*m^2]
    radio = 0.35; %Radio de la ruda trasera [m] Promedio según https://www.zikloland.com/la-potencia-tan-importante/ 40.38N
    Ec = 0.97; %Eficiencia [%/100]
    Aj=0.2;

    temp = 19; hum = 50; phPA = 1025;% Temperature(°C), Humidity (%), Pressure(PA) 
    p = ((0.34848*phPA)-(0.009*hum)*exp(0.061*temp))/(273.15+temp);%Air density [kg/m^3]
    Va = 0; %Ya que es arranque, Velocidad del Aire 0 m/s
    Vx = Va + Vg;
    Cd = 1.1;% Coeficiente de drag [NONE] 
    Area = 0.51; %Area de inclinacion ciclista [m]
    Crr = 0.006;%Coef RollingResis [NONE] MTB:0.012/0.014(Ref2) - Clinch:0.004 BikeCalculator

%%
% FILTROS
altura = xlsread(filename, 'E:E'); %Altura Cementerio
distancia = xlsread(filename, 'F:F'); %Distancia Cementerio

%altura = xlsread(filename, 'E:E'); %Altura Barrios Unidos
%distancia = xlsread(filename, 'F:F'); %Distancia Barrios Unidos

distanciaMetros = distancia .* 1000;
SumaPesos = 7;

coeficientes= ones(1, SumaPesos)/SumaPesos;
AlturaFIL= filter(coeficientes,1,altura);
%AlturaFIL(AlturaFIL<2200) = 2550

diff1 = diff(AlturaFIL);
diff1(diff1>1.2)=0.1;
DIFF1=[diff1;0];
diff1filtrada = filter (coeficientes,1,DIFF1);

diff2 = diff(diff1filtrada);

diff2(abs(diff2)>0.5)=0;
DIFF2=[diff2;0];

%plot(DIFF2)

%%
%División del recorrido

[PicosV,PicosPos]=findpeaks(abs(DIFF2),'MinPeakDistance',8,'MinPeakHeight',0.03);
PicosPos = PicosPos - 2; %Ajuste de posiciòn

%%


Trayectos = length(PicosPos);
ValAltura = AlturaFIL(PicosPos);

for i = 1:Trayectos
    if ValAltura(i) < 2000
        ValAltura(i) = ValAltura(i+1)+0.02
    end
end

%ValAltura = ValAltura.'
ValAltura = [altura(1);ValAltura]
%ValAltura = ValAltura.'
ValDistancia = distanciaMetros(PicosPos);
ValDistancia = [0;ValDistancia]
%ValAltura = ValAltura.'
%ValDistancia = ValDistancia.'
%PicosPos = [0 PicosPos];
%PicosV = [0 PicosV];

%segmentos = table(AlturaFIL(PicosPos),distanciaMetros(PicosPos),PicosPos,PicosV)

for i = 1:Trayectos
    PendienteP100(i)  = ((ValAltura(i+1)-ValAltura(i))*100)/(ValDistancia(i+1)-ValDistancia(i))
    if PendienteP100(i)<0
        PotenciaSlope(i) = -46.287*(abs(PendienteP100(i))).^(0.6612)
    else
    PotenciaSlope(i)=46.287*(abs(PendienteP100(i))).^(0.6612)
    end
end
PendienteP100 = [PendienteP100 0];
PendienteP100 = PendienteP100.'
PotenciaSlope = [PotenciaSlope 0];
PotenciaSlope = PotenciaSlope.'

segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100)
%%
% POT_SLOPE CARACTERIZACIÓN
Slopee = 1:15; % Pendiente en %
%Slopee
DeltaVel = [20 17 14 13 13 12 11 10 10 9 9 9 9 8.5 8.5]; % En km/h

potencia = (DeltaVel./3.6).*800.*sind(atand(Slopee./100)); % En W
%potencia

%plot(Slopee,potencia)

%segmentos = table(AlturaFIL(PicosPos),distanciaMetros(PicosPos),PicosV,PicosPos,PotenciaTrayecto,PendienteP100)


%%
%Velocidad Objetivo


for i = 1:Trayectos
    PentienteRad(i) = atan(PendienteP100(i)/100)
    VelObj(i) = PotenciaSlope(i)/(mt*g*sin(PentienteRad(i))) %Velocidad Objetivo en m/s
    if VelObj(i) > 6.944
       VelObj(i) = 6.944;
    end
    i=i+1

end

PentienteRad = [PentienteRad 0];
PentienteRad = PentienteRad.'
VelObj = [VelObj 0];
VelObj = VelObj.'
VelObj(isnan(VelObj)) = 0;


segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100,PentienteRad,VelObj)

%%
%Aceleración

for i = 1:Trayectos-1
    DeltaVelObj (i) = VelObj(i+1)-VelObj(i)
    %DeltaVelObj (1) = VelObj(1)
end

DeltaVelObj = [VelObj(1) DeltaVelObj];
DeltaVelObj = [DeltaVelObj 0];
DeltaVelObj = DeltaVelObj.'
%DeltaVelObj(isnan(DeltaVelObj)) = 0;

segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100,PentienteRad,VelObj,DeltaVelObj)

for i = 1:Trayectos
    x_o(i) = 5*(1 - (abs(DeltaVelObj(i))*3.6)/25) + 3; %Velocidad en km/h
end
x = 0:.2:16;
y = 25*sigmf(x,[1.2,3]);
a1 = diff(y/3.6);

%plot(x(1:end-1),a1)

x = 0:.2:16;
y = 20*sigmf(x,[1.0,4]);
a2 = diff(y/3.6);

%plot(x(1:end-1),a2)

x = 0:.2:16;
y = 15*sigmf(x,[0.8,5]);
a3 = diff(y/3.6);

%plot(x(1:end-1),a3)

x = 0:.2:16;
y = 10*sigmf(x,[0.6,6]);
a4 = diff(y/3.6);

%plot(x(1:end-1),a4)

A = [a1; a2; a3; a4];
i = 0;
%idx(i) = find(x>x_o(i),1); % índice del vector x

for i = 1:Trayectos
    idx(i) = find(x>x_o(i),1); % índice del vector x
    if x_o(i) < 3
        Av(i) = max(max(A)); %Aceleración gracias a los cambios de velocidad
    elseif x_o > 7
        Av(i) = 0.02;
    else
        Av(i) = max(A(:,idx(i)));
    end
end

Av = Av.*sign(DeltaVelObj(1:end-1))';%cambio de signo para la aceleracion

x_o = [x_o 0];
x_o = x_o.'

Av = [Av 0];
Av = Av.'

%segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100,PentienteRad,VelObj,DeltaVelObj,x_o,Av)

%%
%Aceleración gracias a la pendiente

Ap = g*sin(atan(PentienteRad))

%segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100,PentienteRad,VelObj,DeltaVelObj,x_o,Av,Ap)
%%
%Velocidad Instantánea

%Delta distancia
for i = 1:Trayectos
    DeltaDistancia (i) = ValDistancia(i+1)-ValDistancia(i)
    %DeltaVelObj (1) = VelObj(1)
end

DeltaDistancia = [DeltaDistancia 0];
DeltaDistancia = DeltaDistancia.'


%Tiempo aceleración
tiempoacc = (abs(DeltaVelObj))./(abs(Av+Ap))


%Distancia aceleración
for i = 1:Trayectos
    distancia_acc = (0.*tiempoacc)+(((Av+Ap).*tiempoacc.^2)./2)
end


%Tiempo arranque inicial
tiempo_arr = VelObj(1)./(Av(1)+Ap(1))
distancia_arr = ((Av(1)+Ap(1))*tiempo_arr^2)/2

%Distancia crucero

distancia_cru = DeltaDistancia-distancia_acc

%Tiempo crucero
tiempocru = distancia_cru./VelObj
tiempocru(length(tiempocru)) = []


%segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100,PentienteRad,VelObj,DeltaVelObj,x_o,Av,Ap,DeltaDistancia,tiempocru,tiempoacc,distancia_acc,distancia_cru)

%tiempo total

for i = 1:Trayectos
    if i>1
        Ttotal(i) = tiempocru(i) + tiempoacc(i) + Ttotal(i-1)
    %DeltaVelObj (1) = VelObj(1)
    end
    Ttotal(1) = tiempocru(1) + tiempoacc(1) 
end

Ttotal = [Ttotal 0];
Ttotal = Ttotal.'

%segmentos = table(ValAltura,ValDistancia,PotenciaSlope,PendienteP100,PentienteRad,VelObj,DeltaVelObj,x_o,Av,Ap,DeltaDistancia,tiempocru,tiempoacc,Ttotal,distancia_acc,distancia_cru)

%Tiempo con Aceleraciones


for i = 2:3
        if i == 3 %%& j ==Trayectos
            break
        end
    for j = 1:Trayectos
     TT(1) = tiempo_arr
        
        if i>1
            TT(i) = TT(i-1) + tiempocru(j)
            j = j+1
            TT(i+1) = TT(i) + tiempoacc(j)
            i = i+2
       end
    end
end

TT(Trayectos*2+1) = []

%Por si algo:
%TT(1) = tiempoacc(1)
    %TT(2) = TT(1) + tiempocru(1)
    %TT(3) = TT(2) + tiempoacc(2)
    %TT(4) = TT(3) + tiempocru(2)
    %TT(5) = TT(4) + tiempoacc(3)
    %TT(6) = TT(5) + tiempocru(3)
    %TT(7) = TT(6) + tiempoacc(4)
    %TT(8) = TT(7) + tiempocru(4)
    %TT(3) = TT(2) + tiempoacc(2)


 %for i = 1:Trayectos
    %for j = 1:Trayectos*2
    %VelObjGrafica = kron(VelObj(i), ones(round(tiempocru(j)),1))
    %resultadofinal =reshape(VelObjGrafica,1,size(VelObjGrafica,1)*size(VelObjGrafica,2))
    %end
 %end

%%
%Potencia Instantánea+

Pad = 0.5*p*Cd*Area*VelObj.^3; %Air Power [W - Kg*m^2/s^3] 
Pwr = 0.5*p*Fw*VelObj.^3;%Wheel rotation power [W - Kg*m^2/s^3]
Ps = PotenciaSlope;
Pa = tiempoacc.*Av.^2;

PtotalAcelerando = Pad+Pwr+Ps+Pa
%PtotalAcelerando = PtotalAcelerando.'
PtotalCrucero = Pad+Pwr+Ps

PtotalAcelerando(Trayectos+1) = []
PtotalCrucero(Trayectos+1) = []



%PtotalCrucero = PtotalCrucero.'
PtotalSumada(1:2:(length(PtotalAcelerando)+length(PtotalCrucero))-1) = PtotalAcelerando 
PtotalSumada(2:2:(length(PtotalAcelerando)+length(PtotalCrucero))) = PtotalCrucero 
%Einst = m*g*Altura

PtotalSumada = PtotalSumada.'

%PtotalCrucero = [PtotalCrucero; 0]
%PtotalAcelerando = [PtotalAcelerando; 0]

%VelInst = table(VelObj,tiempocru,Av,tiempoacc)
%TablaAvances = table(PotenciaSlope,VelObj,DeltaVelObj,Av,tiempocru,tiempoacc,PtotalAcelerando,PtotalCrucero)

%plot(TT,PtotalSumada)


%nexttile
%plot(Ttotal,Ptotal)

%nexttile
%plot(Ttotal,VelObj)

%nexttile
%plot(Ttotal,ValAltura)

%%
%Cálculo Velocidad Instantánea y Potencia Instantánea
Av(length(Av)) = []
tiempoacc(length(tiempoacc)) = []
Ap(length(Ap)) = []
t_cru = tiempocru;
t_a = tiempoacc;

%v_a = round(tiempoacc).*Av; 

p_cru = PtotalCrucero
P_ = [];
P_drag = [];
v_cru = VelObj;
v_a = [];
V_ = [];
V_1 = [];
for i = 1:Trayectos
    if i == 1
    v_a = [(Av(i)+Ap(i))*sign(Av(i))*(1:1:round(t_a(i)))] %+ v_cru(i)  
    %v_a1 = v_a/3.6
        for j = 1:length(v_a)
%              if v_a(j) > 6.944
%                 v_a(j) = (6.944)-(v_a(j)-(6.944));
%              end

            if v_a(j) > 6.944 & Ap(i)>0 
               v_a(j) = (6.944)-(v_a(j)-(6.944));

            elseif Ap(i)<0 & v_a(j)>8.33
                   %v_a(j)=8.33
                   v_a(j) = (6.944)-(v_a(j)-(6.944));
                   v_cru(i)= 8.33
  
%             elseif Ap(i)>0 & v_a(j)<8.33
%                    %v_a(j)=8.33
%                    %v_a(j) = (6.944)-(v_a(j)-(6.944));
%                    v_cru(i)= 8.33
            end

        V_ = [V_; v_a(j)*3.6]
        V_1 = V_/3.6
        end
        V_ = [V_; v_cru(i)*3.6*ones(round(t_cru(i)),1)]
        
    end
    if i > 1
    v_a = [(Av(i)+Ap(i))*sign(Av(i))*(1:1:round(t_a(i)))] + (v_cru(i-1))
    %v_a1 = v_a/3.6
        for j = 1:length(v_a)
%              if v_a(j) > 6.944
%                 v_a(j) = (6.944)-(v_a(j)-(6.944));
%              end
            if v_a(j) > 6.944 & Ap(i)>0 
               v_a(j) = (6.944)-(v_a(j)-(6.944));
  
            elseif Ap(i)<0 & v_a(j)>8.33
                   v_a(j)=8.33
                   %v_a(j) = (6.944)-(v_a(j)-(6.944));
                   v_cru(i)= 8.33
            end
        V_ = [V_; v_a(j)*3.6]
        V_1 = V_/3.6
        end
        V_ = [V_; v_cru(i)*3.6*ones(round(t_cru(i)),1)];
       
    end
end

%%Cálculo Potencia Instantánea
p_cru = PtotalCrucero
v_cru = VelObj;
v_a = [];
VA = [];
P_ = [];
for i = 1:Trayectos
    if i == 1
        
            v_a = [(Av(i)+Ap(i))*sign(Av(i))*(1:1:round(t_a(i)))] %+ v_cru(i)

            for k = 1:length(v_a)
                if v_a(k) > 6.944
                   v_a(k) = 6.944-(v_a(k)-6.944);
                end
%                 VA = [VA; v_a(k)]
            end
            P_aireAcc = 0.5*p*Cd*Area*v_a.^3 %Potencia del aire para velocidad instantánea en aceleración
            P_rozamientoAcc = 0.5*p*Fw*v_a.^3
            
        
        P_Total_Acc = P_aireAcc+P_rozamientoAcc+Ps(i)+Pa(i)
        for j = 1:length(P_Total_Acc)
            
        P_ = [P_; P_Total_Acc(j)]
        E_ = cumsum(P_)
        end
        P_ = [P_; p_cru(i)*ones(round(t_cru(i)),1)]
        E_ = cumsum(P_)
    end
    if i > 1
        
             v_a = [(Av(i)+Ap(i))*sign(Av(i))*(1:1:round(t_a(i)))] + v_cru(i-1)
           
            for k = 1:length(v_a)
                if v_a(k) > 6.944
                   v_a(k) = 6.944-(v_a(k)-6.944);
                end
%                 VA = [VA; v_a(k)]
            end
            P_aireAcc = 0.5*p*Cd*Area*v_a.^3 %Potencia del aire para velocidad instantánea en aceleración
            P_rozamientoAcc = 0.5*p*Fw*v_a.^3
        
        P_Total_Acc = P_aireAcc+P_rozamientoAcc+Ps(i)+Pa(i)
        for j = 1:length(P_Total_Acc)
            
        P_ = [P_; P_Total_Acc(j)]
        E_ = cumsum(P_)
        end
        P_ = [P_; p_cru(i)*ones(round(t_cru(i)),1)];
        E_ = cumsum(P_)
    end
end

%%Gráfica Air Drag
p_cru = PtotalCrucero
v_cru = VelObj;
v_a = [];
VA = [];
P_drag = [];
for i = 1:Trayectos
    if i == 1
        
            v_a = [(Av(i)+Ap(i))*sign(Av(i))*(1:1:round(t_a(i)))] %+ v_cru(i)
            for k = 1:length(v_a)
                if v_a(k) > 6.944
                   v_a(k) = 6.944-(v_a(k)-6.944);
                end
%                 VA = [VA; v_a(k)]
            end

            P_aireAcc = 0.5*p*Cd*Area*v_a.^3 %Potencia del aire para velocidad instantánea en aceleración
            
        for j = 1:length(P_aireAcc)
            
        P_drag = [P_drag; P_aireAcc(j)]
        end
        P_drag = [P_drag; Pad(i)*ones(round(t_cru(i)),1)]
    end
    if i > 1
        
          v_a = [(Av(i)+Ap(i))*sign(Av(i))*(1:1:round(t_a(i)))] + v_cru(i-1)  
           for k = 1:length(v_a)
                if v_a(k) > 6.944
                   v_a(k) = 6.944-(v_a(k)-6.944);
                end
%                 VA = [VA; v_a(k)]
            end
            P_aireAcc = 0.5*p*Cd*Area*v_a.^3 %Potencia del aire para velocidad instantánea en aceleración
         
        for j = 1:length(P_aireAcc)
            
        P_drag = [P_drag; P_aireAcc(j)]
        end
        P_drag = [P_drag; Pad(i)*ones(round(t_cru(i)),1)];
    end
end

Ttotal(length(Ttotal)) = []

segmentos = table(DeltaDistancia,distancia_acc,distancia_cru)
segmentos2 = table(t_cru,t_a,Ttotal)


TT = [0 TT];

PendienteP100(length(PendienteP100)) = []
%%Gráfica Bonita

% Create figure
figure1 = figure;

% Create subplot
subplot1 = subplot(3,1,1,'Parent',figure1);
hold(subplot1,'on');

% Create plot Velocidad Instantánea
Plot1 = plot(V_,'DisplayName','Velocidad Instantánea','Parent',subplot1,...
    'LineWidth',2);


for i = 1:2:2*(Trayectos)
    line([TT(i) TT(i)],[min(V_) max(V_)],'LineWidth',2,'LineStyle','--','Color',[1 0 0])
end

% Create ylabel
ylabel('Velocidad(km/h)');

% Create xlabel
xlabel('Tiempo(s)');

box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');

legend1 = legend('Velocidad Instantánea');
set(legend1,'Location','southeast');

% Create subplot
subplot2 = subplot(3,1,2,'Parent',figure1);
hold(subplot2,'on');

% Create plot
Plot2 = plot(P_,'DisplayName','Potencia Instantánea','Parent',subplot2,...
    'LineWidth',2,...
    'Color',[0 0.498039215803146 0]);

Plot21 = plot(P_drag,'DisplayName','Potencia Instantánea','Parent',subplot2,...
    'LineWidth',2,...
    'Color',[0.400000005960464 1 0.200000002980232]);

for i = 1:2:2*(Trayectos)
    line([TT(i) TT(i)],[min(P_) max(P_)],'LineWidth',2,'LineStyle','--','Color',[1 0 0])
end

% Create ylabel
ylabel('Potencia(W)');

% Create xlabel
xlabel('Tiempo(s)');

box(subplot2,'on');
grid(subplot2,'on');
hold(subplot2,'off');

legend2 = legend('Potencia Instantánea Total','Air Drag');
%set(legend2,'Location','eastoutside');

% legend3 = legend(Plot21,'Air Drag');
% set(legend3,'Location','eastoutside');

% Create subplot
subplot3 = subplot(3,1,3,'Parent',figure1);

for i= 1:2:2*(Trayectos)
    line([TT(i) TT(i+2)],[ValAltura((i+1)/2) ValAltura((i+1)/2 +1)], 'DisplayName','Altura',...
    'LineWidth',2,...
    'Color',[1 0.843137264251709 0])
end

for i = 1:2:2*(Trayectos)
    line([TT(i) TT(i)],[min(ValAltura) max(ValAltura)],'LineWidth',2,'LineStyle','--','Color',[1 0 0])
    
    for j = 1:Trayectos
         xt = TT(2*j-1)
         txt = [num2str(round(PendienteP100(j),2)) '%']
         alt = ValAltura(1)
         t = text(xt+2,alt,txt)
         set(t,'Rotation',45);
         t.FontName = 'Courier'
         t.FontSize = 8
         
    end
    %pend=PendienteP100(i)
    %txt = [num2str(PendienteP100)]
    
end

% Create ylabel
ylabel('Altura(m)');

% Create xlabel
xlabel('Tiempo(s)');

% Set the remaining axes properties
set(subplot3,'XColor',[0 0 0],'XGrid','on','YColor',[0 0 0],'YGrid','on',...
    'ZColor',[0 0 0]);

legend3 = legend('Altura');
set(legend3,'Location','southeast');

% Create textbox
annotation(figure1,'textbox',...
    [0.87830789274297 0.428310149626146 0.025707778167902 0.0836919576777588],...
    'String',{round(E_(length(E_))/1000),'kJ'},...
    'FontSize',9,...
    'FitBoxToText','off');
