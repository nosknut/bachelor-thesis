# Software

## Hvorfor ros

ROS2 er en industri standard for automatiserte systemer, og har vert i utvikling siden 2007. Det er et rammeverk som har blitt utviklet av mange leverandører og et stort økosystem. Dette resulterer i mange ferdige løsninger som kan brukes til å bygge komplekse systemer raskt og effektivt. Ved å bruke ROS2 sikres en arkitektur som er godt kjent og enkel å vedlikeholde og videreutvikle.

### Hvorfor ros2 humble

ROS2 har mange forskjellige distribusjoner. Under dette prosjektet sier den offisielle ROS2 dokumentasjonen at "Iron Irwini" er den nyeste, men at den har EOL (End of Life) i November 2024 [[kilde]](https://docs.ros.org/en/rolling/Releases.html). Derfor benyttes i stede "Humble Hawksbill", som har EOL i Mai 2027.

## Ros prosjektstruktur

## ROS2 pakker utviklet for dette prosjektet

I dette prosjektet er det ønskelig å føløge industristandarder til høyest mulig grad, samtidig som kundens ønske for enkelhet blir oppfyllt. Derfor består prosjektet hovedsakelig av konfigurasjoner som benytter eksisterende løsninger, med unntak av controller mappings i twig_teleop og en egen diagnostikk og feilsøkings node i twig_hardware.

### twig

Denne pakken inneholder forskjellige launch filer for å starte systemet i forskjellige konfigurasjoner. Pakken fungerer som et felles grensesnitt for oppstart av manipulator stacken, og brukere av manipulatoren skal ved normal operasjon ikke trenge å forholde seg til de andre pakkene direkte.

#### Launchfiler i twig pakken

##### Normal operasjon

###### robot_moveit.launch.py

Starter alle pakker relatert til hardware og moveit. Med andre ord alle andre pakker enn de som er relatert til GUI og user input. Denne noden må kjøres på selve roboten.

###### controller_manager.launch.py (automatisk start)

Starter en ROS2 Control Controller Manager node konfigurert av twig_moveit_config pakken. Denne noden må kjøres på selve roboten. Denne launchfilen er en del av robot_moveit.launch.py og trenger ikke å kjøres manuelt.

###### servo_teleop.launch.py

Starter Moveit Servo og teleop noden som trengs for å konvertere joystick input til leddhastigheter. Denne noden kan kjøres hvor som helst.

###### foxglove.launch.py

Starter en normal foxglove bridge, som trengs for at foxglove applikasjonen skal kunne snakke med ROS2 systemet. Den er gjort tilgjengelig gjennom twig pakken for enkel installasjon og tilgang. Denne noden kan kjøres hvor som helst.

###### joy.launch.py (foxglove alternativ)

Starter en normal joy node konfigurert for å publisere på twig_joy topicet. Dette er et alternativ til å publisere joy meldinger gjennom foxglove. Denne noden må kjøres på samme PC som joysticken er tilkoblet.

##### Diagnostikk

###### diagnostics.launch.py

Starter en diagnostikk node som kommuniserer direkte med hardware, og en teleop node for joystick kontroll. Den kan ikke kjøres parallellt med andre launchfiler som bruker hardware, og er derfor kun tilgjengelig for diagnostikk og feilsøking. Formålet med denne noden er å gjøre tilgjengelig all informasjon til feilsøkingsformål, som ellers ville vært overflødig for normal operasjon. Noden fungerer også som en manuell backup løsning i tilfelle software feil skulle skapt problemer i det større systemet. Noden må kjøres på selve roboten, og krever at en annen node publiserer joystick data på twig_joy topicet.

###### rviz.launch.py

Starter en ren Rviz node. Denne kan være nyttig for å visualisere den interne tilstanden til roboten. Denne noden vil kun være nyttig med moveit stacken og vil dermed ikke virke med diagnostics.launch.py. Noden må kjøres på en PC med skjerm.

##### Utvikling

###### moveit_demo.launch.py

Starter en ren software demo node med foxglove, Rviz, Moveit, Moveit Servo og en teleop node. Formålet med denne launch filen er å gi en enkel måte å teste ROS2 stacken systemet på uten å måtte sette opp hardware.

### twig_description

Denne pakken inneholder en URDF fil, som beskriver manipulatorens fysiske form, joint typer og joint limits. Denne beskrivelsen benyttes av blandt annet Moveit2 og Tf2 for å regne ut transformasjoner og baner for roboten.

### twig_moveit_config

Denne pakken inneholder en konfigurasjon generert av Moveit Setup Assistant, og brukes til å starte Moveit stacken. Den inneholder bland annet flere konfigurasjoner for baneplanlegging, en kollisjonsmesh basert på robotbeskrivelsen i twig_description, TF2 noder, kinematikk osv.

### twig_hardware

Denne pakken inneholder hardware drivere, "Hardware Interfaces" for ROS2 Control og en ROS2 node for diagnostikk og feilsøking. I tillegg inneholder den også firmware og alle konfigurasjoner relatert til firmware, drivere og kommunikasjon mellom dem. Firmware inngår ikke i ROS2 Stacken, men er gruppert under denne noden siden firmware og drivere skrives sammen.

### twig_servo

Denne pakken inneholder konfigurasjon for Moveit Servo noden som brukes i prosjektet.

### twig_teleop

Denne pakken indeholder noder som basert på joystick meldinger sender kommandoer til twig_hardware nodene gjennom topics og services.

## Hva er

### Topics

ROS2 noder kommuniserer mellom hverandre i hovedsak gjennom topics. Et topic er en navngitt kanal hvor meldinger av en konkret form sendes av en eller flere noder og mottas av en eller flere andre noder. Topics egner seg til kommunikasjon som oppdateres ofte, for eksempel hastighetskommandoer, eller som skal deles ut til mange subscribers som for eksempel systemtilstander. Topics følger en push modell, hvor noder som subscriber til et topic venter på at en annen node publisher en melding. Det er ikke mulig for en subscriber å etterspørre informasjon. Dette er en av grunnene til at ROS2 også har services.

### Services

ROS2 Services er en annen måte å kommunisere med ROS2 noder på. Topics er ment til å overføre hyppige meldinger, men gir ingen tilbakemelding på resultatet. En tjeneste er ment til mer skjelden bruk, og gir en tilbakemelding på handlinge. I tilfellet av manipulatoren blir tjenester brukt til å aktivere og deaktivere releene på hver enkelt servo, og for å kvittere alarmtilstander på hardware. En ROS2 node kan ha flere tjenester samtidig som den også har andre noder. Tjenester følger en pull modell, hvor en node ved behov kan etterspørre informasjon eller en handling fra en annen node, og motta en respons.

### Parameter Server

ROS2 har en standardisert parameter server innebygd i alle noder, som kan brukes til å redigere parametre i sanntid. Dette er spesielt nyttig til for eksempel PID kontrollere, hvor det er ønskelig å kunne justere på parametre uten å måtte starte systemet på nytt. Alle parametre kan til slutt lagres in konfigurasjonsfiler og legges ved i den ferdige ROS2 pakken.

### ROS2 Control

ROS2 Control er en ROS2 pakke som håndterer hardware drivere og kontrollere. Denne pakken er designet for å være en standardisert måte å kommunisere med hardware på. Dette oppnås ved å isolere all kode som snakker med fysisk hardware i "Hardware Interfaces", som standardiseres de tre mest vanlige grensesnittene: Actuator, Sensor og System. PID kontrollere og andre kontrollere kan deretter implementeres i "Controller Interfaces", som sammens med "Hardware Interfaces" kombineres basert på standardiserte konfigurasjonsfiler.

#### Mocked Components

ROS2 Control handler i hovedsak om å isolere ROS2 utvikling fra hardware. For å kunne simulere eller teste ROS2 stacken uten å ha en fysisk robot tilgjengelig har pakken Mocked Components, som er generiske implementasjoner av actuator og sensor hardware interfaces. Dette muliggjør grundig utvikling og testing av for eksempel PID kontrollere og avanserte navigasjonssystemer i Docker og Gazebo, uten at det fysiske systemet må være involvert.

### Robot State Publisher

Robot State Publisher (RSP) er en node som bruker robot beskrivelsen (URDF) fra twig_description til å beregne fremoverkinematikken til roboten basert på /joint_states topicet som publiseres av "joint_state_broadcaster/JointStateBroadcaster" fra ROS2 Control som er konfigurert i twig_moveit_config. Dette er en viktig node for Moveit2, som trenger å vite robotens posisjon og orientasjon for å kunne planlegge baner. TF2 kan bruke denne informasjonen til å beregne relative transformasjoner mellom forskjellige koordinatsystemer.

### Tf2

TF2, eller transform 2, brukes til å regne ut transformasjonene mellom forskjellige koordinatrammer. Dette blir nyttig for kunden i senere tid, når systemet skal bli helautomatisk, og manipulatoren skal flyttes til en posisjon basert på ekkolokasjon eller video.

### Moveit2

Moveit2 brukes for å generere baner for roboten. Dette blir ekstra viktig for kunden i senere tid når dronen skal automatiseres fullstendig, og derfor må kunne planlegge alle bevegelser på egenhpnd.

### Moveit Servo

Moveit Servo er en ROS2 pakke som genererer posisjonskommandoer til Moveit2 basert på en ønsket leddhastighet. På denne måten kan en operatør manuellt styre roboten gjennom det samme systemet som ved helautomatisk operasjon. Fordelen ved å "jogge" individuelle ledd gjennom dette systemet fremfor en enkel hjemmesnekret ROS2 node er at Moveit Servo også inkluderer automatisk singularitets, endepunkts og objekt avoidance. Dette skjer ved å eksponensielt redusere leddhastighetene når roboten nærmer seg en uønsket tilstand. Moveit Servo gjør systemet derfor betydelig mer operatørvennlig på en måte som ellers hadde tatt ekstremt mye tid å implementere på egenhånd.

### Rviz2

Rviz2 benyttes for å visualisere og teste robotens Moveit2 og Moveit Servo stack. Grunnet et over simplistisk design og mangel på Windows støtte er dette verktøyet derimot ikke godt egnet til normal operasjon.

### Foxglove

Foxglove er et verktøy for å lage dashboards for bland annet ROS2. Dette fungerer som et mer brukervenlig alternativ til Rviz2, og egner seg bedre til å designe operatørpanel. Til gripperen brukes foxglove i hovedsak til å publisere joystick data til ROS2 fra spillkontrollere. Dette har fungert som et svert godt alternativ under utvikling i docker, siden det ofte er vanskelig å få tilgang til USB enheter i docker gjennom WSL. Foxglove kan også brukes til å visualisere data fra ROS2, og for å sette ROS2 parametre under testing.

### Firmware

Firmware inneholder kode som kommuniserer direkte med fysiske komponenter. Dette gjelder å justere servo hastighet, aktivere og deaktivere releene, lese av strømsensorer og lese av posisjon gjennom encoderene. Firmware er skrevet i C++ og kjører i dette tilfellet på en Arduino Nano. Den er skrevet for å være så enkel, robust, generisk og konfigurerbar som mulig, for å redusere antall endringer som trengs under testing. For å isolere kode som angår de forskjellige komponentene, brukes objektorientert programmering.

#### Watchdog timer

Siden firmware skal kjøre på en undervannsdrone er den å anse som en kritisk prossess. For å forhindre at systemet blir ødelagt om firmware for hvilken som helst grunn fryser opp benyttes en watchdog timer som fullstendig restarter microkontrolleren etter en bestemt tidsperiode. Dette er en enkel og effektiv måte å sikre mot feiltilstander som kan oppstå ved kommunikasjonsfeil eller andre uforutsette hendelser.

#### Freertos

Ved frakobling kan encoderene fryse programmet, og det er derfor risiko for at systemet ikke lenger vil være operativt. Watchdog Timeren løser allerede denne oppgaven, men siden dette er et kjent problem er det ønskelig å løse det på en mer direkte måte. Ved bruk av FreeRTOS kan encoder koden isoleres i en egen tråd med lavere prioritet enn hovedtråden. Dette vil gjøre at hovedtråden kan fortsette å kjøre selv om encoderen fryser. Dette vil også gjøre det enklere å feilsøke problemet, siden det kan observeres at encoderene ikke lenger publiserer data, men resten av systemet fungerer som normalt. For å reduserer kompleksiteten i systemet har freertos koden blitt isolert i egne klasser.

#### Session Ids

Ved noen feil kan driver eller firmware starte på nytt. For å detektere denne feiltilstanden benyttes en session id, at tall mellom 1 og 10 000, som genereres av firmware ved oppstart og legges ved sensor data. Driveren mottar denne id'en og sender den tilbake med alle kommandopakker. Hvis en session id ikke stemmer vil firmware ignorere kommandoen. Hvis en kommando med riktig session id ikke mottas innen en spesifisert tidsperiode vil firmware anta at driveren har krasjet og nullstille seg selv. Dette er en enkel måte å sikre mot feiltilstander som kan oppstå ved kommunikasjonsfeil eller andre uforutsette hendelser. Kvittering av denne feiltilstanden skjer gjennom en Trigger Service på firmware noden. Ved bruk at twig_teleop noden trigges denne tjenesten ved å trykke på deaktiveringsknappen for releene. Alternativt kan autokvittering av slike tilstander konfigureres i twig_hardware noden.

#### Remote Configuration

Firmware tar tid å flashe, og det er derfor upraktisk å justere parametre ved å kompilere og laste opp ny kode. For å effektivisere denne prosessen sendes alle parametre over I2C koblingen. På denne måten kan også firmware konfigureres gjennom driverene, uten å oppdatere selve koden. Disse innstillingene ligger under twig_hardware ros noden.

#### Timed Fuses

Servoene som benyttes i prosjektet skal i følge produsenten ikke kjøres mot full stopp i mer enn 5-10 sekunder [[kilde]](https://www.bluetrailengineering.com/product-page/underwater-servo-ser-20xx). For å unngå dette er det montert en strømsensor på forsyningslinjen til hver enkelt servo. Koden som overvåker dette ligger i firmware for å sikre mot feil i det mer komplekse ROS2 systemet, og vil koble ut releene og sette servoene i stasjonær tilstand om strømmen overstiger en spesifisert grense i en spesifisert tidsperiode. Servoene vil beholde denne tilstanden frem til en konfigurerbar nedkjølingsperiode er fullført.

#### Max og Min Limits

For å forhindre at manipulatoren kjører leddene mot mekaniske stopp er det satt opp software grenser for hvor langt leddene kan bevege seg. Denne begrensningen er på plass for å sikre mot operatør feil. Av denne grunn er den implementert i selve driveren, siden det uansett er utkoblings og overstrøms sikringer i firmware fra før av for å sikre mot systemfeil. Dette holder kompleksiteten i firmware så lav som mulig, slik at det er enklere å feilsøke og vedlikeholde. Posisjonen måles av 360 graders magnetiske encodere, og for å forenkle implementasjon sentreres posisjonsdata mellom + og - 180 grader. Maks og minimumsvinkler bestemmes basert på denne skalaen, og driveren sender et stoppsignal om en grense overskrides med en hastighetskommando i grenseretningen.

#### Connection Timeout

Hvis firmware ikke blir lest fra eller skrevet til innen et spesifisert tidsintervall, vil releene kobles ut og servoene settes i stasjonær tilstand. På denne måten sikres systemet i tilfeller hvor Raspberry PI fryser på grunn av annen programvare, krasjer, mister forbindelsen eller andre uforutsette hendelser.

### Hvorfor bare .h filer

Det er vanlig å definisjoner i .h filer og implementasjon i .cpp filer. Dette gjør at koden i cpp filene ikke må kompileres på nytt hver gang det gjøres en endring et annet sted i kodebasen. Siden firmware er et lite isolert prosjekt er det derimot lite å vinne på å gjøre dette. Det ble derfor vurdert å ikke benytte cpp filer for firmware prosjektet. ROS2 koden skal derimot kompileres sammens med andre ROS2 pakker og prosjekter, og det er derfor viktig i den sammenheng å overholde slike konvensjoner for å holde den allerede høye kompileringstiden så lav som mulig.

## Raspberry Pi

### mDNS

Multicast DNS (mDNS) er en protokoll som lar enheter på et lokalt nettverk oppdage hverandre uten en DNS server. Dette er spesielt nyttig for å oppdage enheter som ikke har en fast IP adresse, som for eksempel en Raspberry Pi basert på et "hostname". Denne tjenesten er installert som standard på de fleste Raspberry PI OS distribusjoner.

### Ubuntu

Ubuntu er operativsystemet som ROS2 er designet for. ROS2 er også tilgjengelig for andre OS, men ofte krever dette mye ekstra arbeid. Ubuntu versjonen brukt i prosjektet er "Ubuntu Linux - Jammy Jellyfish (22.04)", som også er den anbefalte versjonen for ROS2 Humble.

### Pi Tunnel

pitunnel.com er en tjeneste som kan kobles til en Raspberry Pi for å gjøre en terminal tilgjengelig over internett. Dette gjør feilsøking enklere om det skulle være problemer med å oppdage den over mDNS.

## Workspace

For å kunne utvikle ROS2 prosjekter kreves en omfattende installasjonprossess av Ubuntu, en rekke toolchains og et utviklingsmiljø (IDE). For å unngå at denne prossessen må gjenntas for alle som skal jobbe på prosjektet, er det lagt mye innsats i å designe et ROS2 workspace som er enkelt i bruk og tilpasset til akkurat dette prosjektet. Flere av konfigurasjonene er inspirert av prosjektet "vscode_ros2_workspace" av Allison Thackston [[kilde]](https://github.com/athackst/vscode_ros2_workspace). Andre deler av konfigurasjon kommer fra den offisielle dokumentasjonen til Docker og VSCode.

### Docker

Docker er et system som gjør det mulig å pakke en applikasjon og alle dens avhengigheter i en container. Denne containeren kan baseres på en spesifik distribusjon av linux, og kjøres på hvilken som helst maskin som støtter docker. Dette forenkler drastisk oppsettet for å kjøre prosjektet på forskjellige maskiner, siden docker er uavhengig av operativsystemet som kjører det. Dette er spesielt nyttig for utvikling, siden prosjektet kan utvikles på platformen som er nermere det faktiske systemet det til slutt skal kjøre på. I dette tilfellet er Docker hovedsakelig brukt for å standardisere utviklingsmiljøet.

### VSCode Devcontainer

ROS2 krever ikke bare en spesifik versjon av Ubuntu, men også flere andre programmer og konfigurasjoner for å fungere. For å redusere arbeidsmengden som kreves for å installere ROS2 utviklingsmiljøet på en ny PC benyttes devcontainers. I ".devcontainer" mappen beskrives med konfigurasjonsfiler en Ubuntu container som er ferdig konfigurert med VSCode innstillinger og extensions, ferdig installert ROS2, Python og C++ toolchain. I tillegg er devontaineren konfigurert til å viderekoble filsystemmapper fra WSL2 som gir devcontaineren tilgang til GPU og display drivere, USB devices, shared memory for Intra Process Communication (IPC), nettverkskonfigurasjon for å skjule container barriæren under utvikling og selve workspace mappen som brukes under utvikling.

### WSL2

Windows Subsystem for Linux 2 (WSL2) er en funksjon i Windows som gjør det mulig å kjøre Linux distribusjoner på Windows. Dette trengs i tilegg til docker for å gi docker tilgang til USB enheter, grafikkort og grafikk drivere. Dette gjør det mulig å kjøre GUI applikasjoner som Gazebo og Rviz direkte i Docker på windows uten å installere annet enn Docker og WSL2.

### VSCode Tasks

ROS2 inneholder mange lange kommandoer som er strevsomt å skrive manuelt. For å effektivisere arbeidsflyten er det derfor laget en rekke tasks i VSCode for å utføre disse kommandoene. Disse tasksene ligger under ".vscode/tasks.json" og kan kjøres ved å trykke "Ctrl + Shift + P" og skrive "Run Task". Dette vil gi en liste over tilgjengelige tasks som kan kjøres. For å gjøre det enda enklere kan tasksene også kjøres ved å trykke "Ctrl + Shift + B" og velge ønsket task. Dette vil kjøre den siste kjørte tasken, eller gi en liste over tilgjengelige tasks om ingen har blitt kjørt enda. For videre forenkling er det også forhåndsinstallert en VSCode Extention som heter "Task Buttons", som også gjør de mest hyppige kommandoene: "ROS2 Install Dependencies", "ROS2 Build" og "ROS2 Purge" tilgjengelige som knapper i VSCode nederst på statuslinjen.

### Github

GitHub brukes til å lagre kildekoden i prosjektet. Dette gjør det enkelt å samarbeide, dele kode og holde oversikt over endringer. GitHub Actions brukes til å kjøre tester og bygge docker images automatisk ved push til master branchen.

#### --skip-worktree

I dette prosjektet gir det mening å inkludere en default konfigurasjon for IDE oppsettet. Disse konfigurasjonene er derimot ofte personaliserte til hver enkelt bruker. For å unngå at endringer på disse konfigurasjonene blir pushet til remote, benyttes kommandoen "git update-index --skip-worktree filename" for å ignorere lokale endringer på disse filene. I workspace repo ligger det to scripts for å gjøre denne oppgaven automatisk. Disse ligger under "scripts/git/" og heter "ignore_local_config_file_changes.bash" og "un_ignore_local_config_file_changes.bash".

### vcstool

Vsctool er et standard verktøy i ROS2 for å hå håndtere flere git repositoryer. Dette verktøyet ble brukt for å kunne legge pakkene inn i egene git repositoryer, og deretter importere dem inn i workspace repoet. På denne måten dekobles pakker som ikke er relevante for hverandre, noe som gjør det enklere å skaffe overblikk og vedlikeholde prosjektet.

## Hvordan

### Installere prosjektet

Den følgende installasjonen tar mindre enn 10 minutter manuelt arbeid, og mellom ett minutt til en time med hands-off automatisk bygging avhengig av hvor rask datamaskinen er.

- Installer VSCode
- Installer Git
- Installer Docker
- Installert Foxglove fra foxglove.dev
- Kjør `git clone https://github.com/nosknut/bachelor-thesis.git`
- Kjør `code bachelor-thesis` for å åpne repo i VSCode
- Kjør `bash ./scripts/git/ignore_local_config_file_changes.bash` for å ignorere lokale endringer på konfigurasjonsfiler.
- Følg installasjonsinstruksene for WSL2 og Docker i prosjektets README.md fil for GUI support gjennom Docker og WSL2.
- Åpne Command Palette og kjør kommandoen: "Dev Containers: Reopen in Container"
  - Dette vil bygge en Docker container basert på konfigurasjonen i ".devcontainer" mappen.
- VSCode vil nå bygge en Docker Container basert på konfigurasjonen i ".devcontainer" mappen.
- Når byggingen er ferdig vil VSCode åpne på nytt i et fullverdig og ferdig konfigurert ROS2 utviklingsmiljø.

Dette vil også fungere for linux. For linux brukere som ikke ønsker docker kan det meste automatisk installeres ved å kjøre `bash ./scripts/ubuntu/install-ros2.bash`. Automatisk sourcing av ROS2 workspace kan konfigureres ved å kjøre `bash ./scripts/configure-workspace.bash`.

### Bygge Prosjektet

- For å installere alle dependencies, trykk på "ROS2 Install Dependencies" knappen nederst i VSCode. Dette kan også oppnås ved å kjøre `bash ./scripts/ros2/install-dependencies.bash` i terminalen.
- For å bygge prosjektet trykk på "ROS2 Build" knappen nederst i VSCode. Dette kan også oppnås ved å kjøre `bash ./scripts/ros2/build-packages.bash` i terminalen. Merk at denne kommandoen kjører med `--symlink-install` som betyr at endring av python og konfigurasjonsfiler ikke krever en ny bygging av prosjektet for å tre i kraft.

### Starte Stacken

#### Kjøring

På roboten, kjør `ros2 launch twig robot_moveit.launch.py` og `ros2 launch twig servo_teleop.launch.py`. På grensesnitt maskinen, kjør `ros2 launch twig foxglove.launch.py` og eventuekt `ros2 launch twig joy.launch.py` om det ikke er ønskelig å bruke foxglove til å publisere joystick data. Til slutt, kjør `ros2 launch twig rviz.launch.py` på en PC med skjerm for å visualisere de interne sensortilstandene til roboten. Dette er derimot ikke nødvendig da foxglove også kan brukes for å visualisere posisjonen til hver enkelt joint.

#### Utvikling

For å kjøre en demo av hele ROS2 stacken uten hardware visualisert i Rviz2, kjør `ros2 launch twig moveit_demo.launch.py`. GUI appllikasjonen Rviz2 vil åpne seg og vise en 3D modell av manipulatoren. For å kjøre en demo med hardware, kjør `ros2 launch twig robot_moveit.launch.py`. Denne kommandoen vil starte alle ROS2 noder som er nødvendige for å kjøre manipulatoren, inkludert hardware noder.

For å kontrollere modellen, åpne foxglove og publiser gamepad data til "/twig_joy" gjennom gamepad extension. Hold inne dødmansknappen "RT" på kontrolleren og bruk venstre og høyre joystick for å styre aksene.

Denne utviklingsstacken er også tilgjengelig som en ren Docker Container. For å kjøre denne containeren uten å åpne VSCode i en devcontainer, åpne repoet i en WSL2 terminal og kjør `docker compose up twig`.

### Konfigurere Prosjektet

Nesten alle pakker i "src" mappen inneholder en "config" mappe med .yaml formarterte konfigurasjonsfiler. Disse config filene importeres av ".launch.py" filene i de respektive "launch" mappene.

Noen pekere på relevante konfigurasjoner:
- firmware og drivere ligger under "twig_hardware/config".
- Keybinds for spillkontrollere ligger under "twig_teleop/config".
- Moveit Servo konfigurasjoner med collision avoidance ligger under "twig_servo/config".
- Inverskinematikk og baneplanlegging ligger under "twig_moveit_config/config".
- URDF beskrivelsen ligger under "twig_description/config".
- ROS2 Control konfigurasjoner ligger under "twig_hardware/config" og "twig_moveit_config/config".


### Importere Prosjektet

Prosjektet kan enten startes manuelt, eller legges til som dependency og startes av launchfiler i et helhetlig prosjekt. Se "twig" pakken som eksempel på hvordan dette gjennomføres.

### Utvikle i Prosjektet

Utvikling kan gjennomføres som alle andre ROS2 prosjekter. Om det er ønskelig kan workspacet brukes. Devcontaineren er bygget lokalt på hver enkelt utviklings PC og kan dermed tilpasses den enkeltes smak og behag. Filene i ".vscode" og ".devcontainer" vil isåfal være av relevanse. Ved bruk av devcontaineren vil alt være ferdig konfigurert, og den normale terminalen kan brukes til å kjøre normale ROS2 kommandoer og ROS2 GUI applikasjoner som Rviz2 og RQT. Devcontaineren kan også brukes til å kjøre Rviz og andre GUI applikasjoner oppimot en fysisk robot på samme nettverk som i en vanlig ROS2 stack. For å unngå mye skriving av kommandoer kan knappene "ROS2 Buld", "ROS2 Install Dependencies" og "ROS2 Purge" på statuslinjen i VSCode brukes under utvilking. Formålet med "ROS2 Purge" er å fjerne alle bygde filer og konfigurasjoner for å starte på nytt om det skulle være nødvendig med en fersk start. Ved større problemer kan devcontaineren bygges på nytt ved å åpne repoet lokalt (ikke i devcontaineren) og kjøre "Dev Containers: Rebuild and Reopen Container" fra Command Palette. Selve repoet vil bli værende, men alle andre konfigurasjoner internt i devcontaineren (inkludert VSCode extentions) vil bli nullstillt, for å fjerne eventuelle problemer som har oppstått på grunn av dette. For å beholde andre konfigurasjoner kan enkeltfiler mountes til det fysiske filsystemet ved å legge de til under "services.ros2.volumes" i ".devcontainer/docker-compose.yml" filen. Merk at disse filbanene refererer til plasseringen i WSL2, ikke på Windows. For de som bruker Ubuntu i stede for Windows refererer filbanene til det fysiske filsystemet.

## Teknologier som ikke ble brukt

Følgende teknologier ble vurdert og fungerte, men ble ikke tatt i bruk i det endelige prosjektet fordi det var enklere eller mer effektivt å bruke andre teknologier.

### Gazebo

Gazebo er en 3D simulator med fysikk motor som ofte blir brukt under utvikling av roboter. Originalt var tanken å benytte Gazebo for å produsere sensor feedback. Det viste seg derimot at Mocked Components fra ROS2 Control og Rviz2 var en bedre løsning. Gazebo har fortsatt en relevanse for mer kompleks systemtesting, men dette ble ikke prioritert.

### Usbipd

Usbipd er en tjeneste som lar deg dele USB enheter over nettverket. Dette verktøyet ble vurdert for å dele usb spillkontroller fra windows operativsystemet inn i WSL, som derfra kunne deles til en docker container. Dette fungerte fint, men krevde litt for mye arbeid for å være praktisk i arbeidsflyten. Dette verktøyet ble erstattet med foxglove, som kan publisere en generisk spillkontroller direkte til ROS2.

## Referanser

- https://docs.ros.org/en/humble/Installation.html
- https://learn.microsoft.com/en-us/windows/wsl/connect-usb
- https://www.bluetrailengineering.com/product-page/underwater-servo-ser-20xx
- https://docs.ros.org/en/rolling/Releases.html
- https://github.com/athackst/vscode_ros2_workspace