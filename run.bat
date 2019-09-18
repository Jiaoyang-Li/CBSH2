@echo off 

set time=60
set map="instances\lak503d"
set agent="instances\lak503dmap"
set output="outputs\lak503d"

for /l %%k in (100,10,100) do ( 
  for /l %%i in (0,1,49) do (
    echo Agent %%k ; Instance %%i
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-CBS.csv -t %time% -s 1 -h NONE -p 0
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-ICBS.csv -t %time% -s 1 -h NONE
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-CG.csv -t %time% -s 1 -h CG
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-DG.csv -t %time% -s 1 -h DG
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-WDG.csv -t %time% -s 1 -h WDG
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-CG-R.csv -t %time% -s 1 -h CG -r 1
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-DG-R.csv -t %time% -s 1 -h DG -r 1
    x64\Release\CBSH.exe -m %map%.map  -a %agent%-%%kagents-%%i.agents -o %output%-%%kagents-WDG-R.csv -t %time% -s 1 -h WDG -r 1
  )
)

pause
    
