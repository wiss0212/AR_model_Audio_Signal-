
[S, Fe ] = audioread('Fichier.wav');% lit et renvoie les donnees echantillonnees dans y
L= length(S);
f = Fe*(0:(L/2))/L;
L= length(S);
%...............................................................................
sound(S,Fe);
%.............................................................................


info = audioinfo('Fichier.wav') ;% renvoie les informations sur le contenu du fichier audio 
fprintf('La Frequence d echantillonnage du signal est de : 8kHz\n');
fprintf('donne l ensemble des donnees de l audio mais surtout le nombre de bits\n');
[S,Fe] = audioread('Fichier.wav', 'native' ); 
whos S

%%
%*************************************************************************%
%*************************************************************************%
% 
%Tracer temporelle du signal
t0=(0:L-1)/Fe; % le vecteur de temps associÃ© Ã  L

figure(1)
subplot(1,2,1);
plot(t0,S);
xlabel('Temps');
ylabel('Amplitude');
title('Représentation temporelle du signal');

%Tracer frÃ©quencielle du signal
f0=(0:L-1)/L*Fe-Fe/2;
S0 = fftshift(fft(S));

subplot(1,2,2);
plot(f0,20*log10(abs(S0)));
xlabel('Fréquence(hz)');
ylabel('Amplitude');
grid on
title('Représentation fréquencielle du signal');

%
%*************************************************************************%
% Essaie d'extraction avec Analyse de fourier
L= length(S);
f = Fe*(0:(L/2))/L;

S_fft = fft(S);
% pic a droite de zero
P2= abs(S_fft/L); 
P1 =  P2(1:L/2+1); 
P1(2:end-1) = 2*P1(2:end-1);

%Affichage
figure (2)
plot(f, P1);
grid on ; 
xlabel('Fréquence (Hz)') ;
ylabel('Amplitude') ;
title('Transformé de Fourier du signal original') ;

%%
.........................................................................%
%*************************************************************************%
%Mise en place de la methode AR

% Filtre passe bande
h = bandpass(double(S),[450,1000],8000);
figure (3); 
subplot(1,2,1) ; plot(S) ;
subplot(1,2,2) ; plot(h) ;

f = Fe*(0:(L-1))/L - Fe/2;
figure(4) 

subplot(1,2,1) ; semilogy(f,abs(fftshift(fft(S)))) ; 
xlabel('Fréquence(hz)');
ylabel('Amplitude'); title (" Representation fréquentielle du signal avant filtarge");
subplot(1,2,2) ; semilogy(f,abs(fftshift(fft(h)))) ; xlabel('Fréquence(hz)');
ylabel('Amplitude'); title (" Representation fréquentielle du signal aprés filtarge");

%sound(h,Fe);

%Application de la Methode AR
 [coef,sigma2]=Coef_AR(h,4); % calcul des coefficients du modele AR 
figure (10)
zplane(1,coef.');
Signal_DSP= 0; 
ordre=4;

for n=1:8000
    Part1 = 1;
    for t=1:ordre
        coef_new=coef(t+1,1)*exp(-2*pi*1i*t*n/8000);
        Part1=Part1+coef_new;
    end
    Signal_DSP =[Signal_DSP sigma2/(abs(Part1).^2)]; % Calcul de la DSP
    dspA= abs(Signal_DSP);
    %sound(dspA,Fe)
end
%
X = zeros (322560,1); 
for i=5 : size(h,1)
    Somme = 0; 
    for j = 1 : ordre
    Somme =  coef(j)*h(i-j);
    X(i)= Somme; 
    end
end     
%Resultats:
figure(5)
k=(0:1/8000:0.5);
plot(k,dspA(1,1:length(k)));
xlabel('Fréquence (Hz)') ;
ylabel('Amplitude') ;
title('DSP du signal');

figure(6) 
cwt(X,8000)
sound(X,Fe);
%%



%.........................................................................%

% Robustesse du systeme:

% On a 3 parametres:
%- La resolution frÃ©quentielle
%- La largeur de fenetre 
%- le pas de decalage de la fenetre 

% % Ecart de frequences minimales
% figure(7) 
% [X2,time,freq]= stft(X,50,Fe,blackman(100000), 10000);
% imagesc(time,freq,abs(X2));
% axis xy;
% xlabel('Temps') ;
% ylabel('Frequence') ;
% colorbar 
% title('Ecart minimal')

% Rapport signal sur bruit


% t = 1:0.01:2;
% figure
% plot(t,X)
% w_noise = wgn(1,101,-20);
% hold on
% plot(t,(x+w_noise))
% legend('Sine Wave','Sine Wave with Noise')
% 
% 
% 
% [y,fs,bits]=wavread('nom de votre fichier.wav');
% m=length(y);
% se=norm(S,2)^2/m; %puissance du signal original
% SNR=input('Entrer la valeur désirée du rapport SNR :');
% ec=se/(10^(SNR/10));
% brui=0.2*sqrt(ec)*randn(m,1);
% ne=norm(brui,2)^2/m;
% RSB=10*log10(se/ne);
% fprintf('Estimated SNR=%f\n',RSB);
% 
% 
% 
% 
% 
