# Parte prática

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/menotti/aoc)

## Simulações

Em geral, as simulações funcionais podem todas ser realizada em ambiente **Linux**. A seguir está o comando de instalação, dependendo de sua distribuição, dos pacotes necessários: `[apt|snap|yum|rpm|dnf|pacman] install iverilog gtkwave gcc-riscv64-linux-gnu binutils-riscv64-linux-gnu qemu-user qemu-user-static`

### Extensões úteis para o VSCode

- [Verilog](https://marketplace.visualstudio.com/items/?itemName=mshr-h.VerilogHDL)
- [WaveTrave](https://marketplace.visualstudio.com/publishers/wavetrace)
- [DigitalJS](https://marketplace.visualstudio.com/items/?itemName=yuyichao.digitaljs)
- [RISCV](https://marketplace.visualstudio.com/items/?itemName=zhwu95.riscv)

## Kit de FPGA

Para sintetizar cada laboratório e fazer o download para a placa, digite `..\make_wannabe.bat` (*Windows*) ou `make -f ../Makefile` (*Linux*) **dentro do diretório** do respectivo laboratório (não [nesta pasta](https://github.com/menotti/aoc/blob/main/labs/), pois os comandos fazem referência ao nível superior). 

### DE0-CV

Consulte o [manual do kit](https://github.com/menotti/aoc/blob/main/labs/DE0_CV_User_Manual.pdf) para compreender como usar cada recurso. Um [arquivo de pinos](https://github.com/menotti/aoc/blob/main/labs/DE0_CV.qsf) é usado para mapear as entradas e saídas.