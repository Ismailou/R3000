--------------------------------------------------------------------------------
-- RISC processor general definitions
-- THIEBOLT Francois le 08/03/04
--------------------------------------------------------------------------------

-- library definitions
library IEEE;

-- library uses
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

-- -----------------------------------------------------------------------------
-- the package contains types, constants, and function prototypes
-- -----------------------------------------------------------------------------
package cpu_package is


-- ===============================================================
-- DEFINITION DE FONCTIONS/PROCEDURES de base
-- ===============================================================

	-- Fonction log2
	--		calcule le logarithme base2 d'un entier naturel, ou plus exactement
	--		renvoie le nombre de bits necessaire pour coder un entier naturel I
	function log2 (I: in natural) return natural;

-- ===============================================================
-- TYPES/CONSTANT DEFINITIONS
-- ===============================================================

------------------------------------------------------------------
-- HARDWARE definitions
------------------------------------------------------------------
	-- define CPU core physical sizes
	constant CPU_DATA_WIDTH	 : positive := 32; 				-- data bus width
	constant CPU_INST_WIDTH	 : positive := CPU_DATA_WIDTH; -- instruction bus width
	constant CPU_ADR_WIDTH	  : positive := 32; 				-- address bus width, byte format

	-- define MISC CPU CORE specs
	constant CPU_WR_FRONT	 : std_logic	:= '1';				-- pipes write active front
	constant PC_WIDTH			   : positive	 := 26;				-- bits pour le PC format mot memoire ??? exp: range of PC in memory block [ we have (2**(6-PCLOW)) memory block ]
	constant PCLOW				     : positive	 := 2; -- log2(CPU_INST_WIDTH/8) = log2(32/8) = log2(4) = 2;

	-- define REGISTERS physical sizes
	constant REG_WIDTH		: positive	  := 5; -- registers address bus with
	constant REG_FRONT		: std_logic	 := CPU_WR_FRONT;

	-- define instruction & data CACHE physical sizes
	constant L1_SIZE		 	: positive	  := 16; -- taille des caches L1 en nombre de mots
	constant L1_ISIZE		 : positive	  := L1_SIZE; -- taille du cache instruction L1 en nombre de mots
	constant L1_DSIZE			: positive	  := L1_SIZE; -- taille du cache donnees L1 en nombre de mots
	constant L1_FRONT			: std_logic	 := CPU_WR_FRONT;

	-- define types/subtypes according to hardware specs
	subtype PC			 is std_logic_vector(PC_WIDTH-1+PCLOW downto PCLOW); -- 29..4 ??? corrected to 27..2 :  the 4 first bits are reserved to index the block memory and the 2 least bit are resered to index bytes in instruction.
	subtype INST		is std_logic_vector(CPU_INST_WIDTH-1 downto 0); -- 31..0
	subtype ADDR		is std_logic_vector(CPU_ADR_WIDTH-1 downto 0); -- 31..0
	subtype DATA		is std_logic_vector(CPU_DATA_WIDTH-1 downto 0);
	subtype REGS		is std_logic_vector(REG_WIDTH-1 downto 0);

  -- constant used as inputs for ALU
  constant EX_ZERO    : DATA := X"00000000";
  constant EX_VAL_16  : DATA := X"00000010";
  
	-- define default values
	constant PC_DEFL : ADDR := conv_std_logic_vector(0,ADDR'length);

------------------------------------------------------------------
-- SOFTWARE definitions
------------------------------------------------------------------
	-- define the basic ALU operations
	-- 	Le fait qu'une operation soit signee ou non sera indique a l'ALU par un signal
	--		supplementaire, ceci dit cela n'affecte que les bits d'etat.
	type ALU_OPS is (ALU_ADD,ALU_SUB,ALU_AND,ALU_OR,ALU_NOR,ALU_XOR,ALU_SLT,ALU_LSL,ALU_LSR);

	-- define the size of datas during memory access
	type MEM_DS is (MEM_8,MEM_16,MEM_32,MEM_64);
	
	--define the control of l'unite des aleas
	type ALEA_CTRL is (ALEA_DETECT, NO_ALEA);
	  
	-- definition des champs d'instructions
	subtype OPCODE	 	is std_logic_vector(31 downto 26);
	subtype RS 			   is std_logic_vector(25 downto 21);
	subtype RT			    is std_logic_vector(20 downto 16);
	subtype RD			    is std_logic_vector(15 downto 11);
	subtype VALDEC		 is std_logic_vector(10 downto 6);
	subtype FCODE		  is std_logic_vector(5 downto 0);
	subtype BCODE		  is std_logic_vector(20 downto 16);
	subtype IMM			   is std_logic_vector(15 downto 0);
	subtype JADR		   is std_logic_vector(25 downto 0);

	---------------------------------------------------------------
	-- definition des constantes de type des instructions

	---------------------------------------------------------------
	-- Instructions de type R : Registre Inst[OPCODE'range]
	constant TYPE_R	: std_logic_vector := "000000" ;

	-- Fonctions associes au TYPE_R Inst[FCODE'range]
	constant ADD 		: std_logic_vector := "100000" ;
	constant ADDU 	: std_logic_vector := "100001" ;
	constant SUB 		: std_logic_vector := "100010" ;
	constant SUBU 	: std_logic_vector := "100011" ;
	constant iAND 	: std_logic_vector := "100100" ; -- i pour differencier l'operateur du meme nom
	constant iOR 		: std_logic_vector := "100101" ; -- i pour differencier l'operateur du meme nom
	constant iNOR 	: std_logic_vector := "100111" ; -- i pour differencier l'operateur du meme nom
	constant iXOR 	: std_logic_vector := "100110" ; -- i pour differencier l'operateur du meme nom
	constant SLT 		: std_logic_vector := "101010" ;
	constant SLTU 	: std_logic_vector := "101011" ;		
	constant LSL 		: std_logic_vector := "000000" ;
	constant LSR 		: std_logic_vector := "000010" ;
	constant JR 		 : std_logic_vector := "001000" ;
	constant JALR 	: std_logic_vector := "001001" ;

	---------------------------------------------------------------
	-- Instruction de Type B : Branchement
	constant TYPE_B 	: std_logic_vector := "000001" ;
	-- Branch associes au TYPE_B Inst[BCODE'range]
	constant BLTZ 		: std_logic_vector := "00000" ;
	constant BGEZ 		: std_logic_vector := "00001" ;
	constant BLTZAL	: std_logic_vector := "10000" ;		
	constant BGEZAL	: std_logic_vector := "10001" ;

	---------------------------------------------------------------
	-- Instructions de type J : Saut
	constant J	 		: std_logic_vector := "000010" ;
	constant JAL	 : std_logic_vector := "000011" ;

	---------------------------------------------------------------
	-- Instruction de type I : Immediat
	constant ADDI 		: std_logic_vector := "001000" ;
	constant ADDIU 	: std_logic_vector := "001001" ;
	constant SLTI 		: std_logic_vector := "001010" ;
	constant SLTIU 	: std_logic_vector := "001011" ;
	constant ANDI 		: std_logic_vector := "001100" ;
	constant ORI 	 	: std_logic_vector := "001101" ;
	constant XORI 		: std_logic_vector := "001110" ;
	constant LUI 		 : std_logic_vector := "001111" ;
	constant LB 		  : std_logic_vector := "100000" ;
	constant LH 		  : std_logic_vector := "100001" ;
	constant LW 		  : std_logic_vector := "100011" ;
	constant LBU 		 : std_logic_vector := "100100" ;
	constant LHU 		 : std_logic_vector := "100101" ;
	constant SB 		  : std_logic_vector := "101000" ;
	constant SH 		  : std_logic_vector := "101001" ;
	constant SW 		  : std_logic_vector := "101011" ;
	constant BEQ 		 : std_logic_vector := "000100" ;
	constant BNE 		 : std_logic_vector := "000101" ;
	constant BLEZ 		: std_logic_vector := "000110" ;
	constant BGTZ 		: std_logic_vector := "000111" ;		

------------------------------------------------------------------
-- HARDWARE Multiplexer and Pipelines registers definitions
------------------------------------------------------------------

	---------------------------------------------------------------
	-- Definition des multiplexeurs dans les etages
	type MUX_ALU_A 	    	is (REGS_QA,REGS_QB,IMMD);
	type MUX_ALU_B 	    	is (REGS_QB,IMMD,VAL_DEC,VAL_16,VAL_ZERO);
	type MUX_REG_DST	    is (REG_RD,REG_RT,R31);
	type MUX_REGS_D	    	is (ALU_S,MEM_Q,NextPC);
 	type MUX_ALU_A_SEND 	is (SRC_MUX_ALU_A,SEND_UNIT_EX,SEND_UNIT_MEM);
  type MUX_ALU_B_SEND 	is (SRC_MUX_ALU_B,SEND_UNIT_EX,SEND_UNIT_MEM);
  type BRANCHEMENT_SOURCE is (BRANCHEMENT_NONE,BRANCHEMENT_BLTZ,BRANCHEMENT_BGEZ,BRANCHEMENT_BLTZAL,
                              BRANCHEMENT_BGEZAL,BRANCHEMENT_BEQ,BRANCHEMENT_BNE,BRANCHEMENT_BGTZ,BRANCHEMENT_BLEZ);
	
	---------------------------------------------------------------
	-- Definitions des structures de controles des etages

	-- Structure des signaux de control de l'etage DI
	type mxDI is record
		SIGNED_EXT	 	: std_logic;							-- extension signee ou non donnee immediate
	end record;	

	-- default DI control
	constant DI_DEFL : mxDI := ( SIGNED_EXT=>'0' );
			
	-- Structure des signaux de control de l'etage EX
	type mxEX is record
		ALU_OP 			  : ALU_OPS; 								-- operation sur l'ALU
		ALU_SIGNED	 : std_logic;							-- operation ALU signee ou non
		ALU_SRCA 		 : MUX_ALU_A;							-- mux pour entree A de l'ALU
		ALU_SRCB 		 : MUX_ALU_B;							-- mux pour entree B de l'ALU
		REG_DST 			 : MUX_REG_DST;					-- mux pour registre destinataire
		BRA_SRC			  : BRANCHEMENT_SOURCE;
		SAUT	       : std_logic;            -- true if the type of current instruction is J
	end record;	
			
	-- default EX control
	constant EX_DEFL : mxEX := ( ALU_OP=>ALU_OPS'low, ALU_SRCA=>MUX_ALU_A'low, ALU_SRCB=>MUX_ALU_B'low,
								 REG_DST=>MUX_REG_DST'low, BRA_SRC=>BRANCHEMENT_SOURCE'low, others=>'0' );
			
	-- Structure des signaux de control de l'etage MEM
	type mxMEM is record
		DC_DS 			 : MEM_DS;							  	-- DataCache taille d'acces 8/16/32/64/...
		DC_RW 			 : std_logic;							-- DataCache signal R/W*
		DC_AS 			 : std_logic;							-- DataCache signal Address Strobe
		DC_SIGNED	: std_logic;							-- DataCache operation signee ou non (lecture)
	end record;	
		
	-- default MEM control
	constant MEM_DEFL : mxMEM := ( DC_DS=>MEM_DS'low, DC_RW=>'1', others=>'0' );
			
	-- Structure des signaux de control de l'etage ER
	type mxER is record
		REGS_W 			: std_logic;							 -- signal d'ecriture W* du banc de registres
		REGS_SRCD	: MUX_REGS_D;							-- mux vers bus de donnee D du banc de registres
	end record;	

	-- default ER control
	constant ER_DEFL : mxER := ( REGS_W=>'1', REGS_SRCD=>MUX_REGS_D'low );
			
	---------------------------------------------------------------
	-- Definition des strucures des registres pipeline
	
	-- Structure du registre EI/DI
	type EI_DI is record
		-- === Data ===
		pc_next 	: std_logic_vector (PC'range);   					-- cp incremente
		inst	 			: std_logic_vector (INST'range);						-- instruction extraite
		-- === Control ===
	end record;		

	-- Structure du registre DI/EX
	type DI_EX is record
		-- === Data ===
		pc_next			: std_logic_vector (PC'range);						  -- cp incremente propage
		rs					: std_logic_vector (REGS'range);						-- champ rs
		rt					: std_logic_vector (REGS'range);						-- champ rt
		rd					: std_logic_vector (REGS'range);						-- champ rd
		val_dec			: std_logic_vector (VALDEC'range);				-- valeur de decalage
		imm_ext			: std_logic_vector (DATA'range);						-- valeur immediate etendue
		jump_adr		: std_logic_vector (JADR'RANGE);						-- champ adresse de sauts
		rs_read			: std_logic_vector (DATA'range);						-- donnee du registre lu rs
		rt_read			: std_logic_vector (DATA'range);						-- donnee du registre lu rt
		code_op			: std_logic_vector (OPCODE'length-1 downto 0);	-- code op
		--code_fonction	: std_logic_vector (FCODE'length-1 downto 0);	-- code fonction
		-- === Control ===
		ex_ctrl			: mxEX;		 -- signaux de control de l'etage EX
		mem_ctrl  : mxMEM;		-- signaux de control de l'etage MEM
		er_ctrl 		: mxER;		 -- signaux de control de l'etage ER
	end record;		

	-- Structure du registre EX/MEM
	type EX_MEM is record
		-- === Data ===
		pc_next 	: std_logic_vector (PC'range);						   -- cp incremente propage
		ual_S				: std_logic_vector (DATA'range);						 -- resultat ual
		rt				 : std_logic_vector (DATA'range);					 -- registre rt propage
		reg_dst		: std_logic_vector (REGS'range);						 -- registre destination (MUX_REG_DST)
		zero     : std_logic;
		-- === Control ===
		mem_ctrl : mxMEM;		 -- signaux de control de l'etage MEM
		er_ctrl 	: mxER;		  -- signaux de control de l'etage ER
	end record;		

	-- Structure du registre MEM/ER
	type MEM_ER is record
		-- Signaux
		pc_next		: std_logic_vector (PC'range);						  -- cp incremente propage
		mem_Q				: std_logic_vector (DATA'range);						-- sortie memoire
		ual_S				: std_logic_vector (DATA'range);						-- resultat ual propage
		reg_dst		: std_logic_vector (REGS'range);						-- registre destination propage

		-- Signaux de control
		er_ctrl		: mxER;		-- signaux de control de l'etage ER propage
	end record;
	
-- ===============================================================
-- DEFINITION DE FONCTIONS/PROCEDURES
-- ===============================================================

-- Si on ne specifie rien devant les parametres...il considere que c'est une variable
-- exemple : procedure adder_cla (A,B: in std_logic_vector;...)
-- ici A et B sont consideres comme etant des variables...
-- Sinon il faut : procedure adder_cla (signal A,B: in std_logic_vector;...)

	-- Fonction "+" --> procedure adder_cla
	function "+" (A,B: in std_logic_vector) return std_logic_vector;

	-- Procedure adder_cla
	procedure adder_cla (A,B: in std_logic_vector; C_IN : in std_logic;
								S : out std_logic_vector; C_OUT : out std_logic;
								V : out std_logic);

	-- Procedure alu
	-- 	on notera l'utilisation d'un signal comme parametres formels de type OUT
	procedure alu (A,B: in std_logic_vector; signal S: out std_logic_vector;
						signal N,V,Z,C: out std_logic; SIGNED_OP: in std_logic;
						CTRL_ALU: in ALU_OPS);

	-- Procedure control
	--		permet de positionner les signaux de control pour chaque etage (EX MEM ER)
	--		en fonction de l'instruction identifiee soit par son code op, soit par
	--		son code fonction, soit par son code branchement.
	procedure control ( OP : in std_logic_vector(OPCODE'length-1 downto 0);
							        F  : in std_logic_vector(FCODE'length-1 downto 0);
							        B  : in std_logic_vector(BCODE'length-1 downto 0);
							        b_condition : in std_logic;
							        j_counter   : in std_logic_vector(1 downto 0);
							        di_halt     : in std_logic;
							        signal ei_flush    : out std_logic;
	                    signal di_flush    : out std_logic;
	                    signal ex_flush    : out std_logic;
							        signal j_increment : out std_logic;
							        signal DI_ctrl	    : out mxDI;		 -- signaux de controle de l'etage DI
							        signal EX_ctrl	    : out mxEX;		 -- signaux de controle de l'etage EX
							        signal MEM_ctrl	   : out mxMEM;	 -- signaux de controle de l'etage MEM
							        signal ER_ctrl	    : out mxER ); -- signaux de controle de l'etage ER
							
	--  Procedure envoi 
  --		Permet de positionner les entrees de l'ALU pour donner la bonne valeur 
  --		commme parametre afin de remedier aux aleas de donnes 
  procedure envoi ( DI_EX_rs : in std_logic_vector(REGS'range);       -- rs register address in the EX stage
                    DI_EX_rt : in std_logic_vector(REGS'range);       -- rt register address in the EX stage
                    EX_MEM_er_ctrl_RESG_W : in std_logic;             -- Write signal for the register banc in the EX_MEM pipeline register
                    EX_MEM_reg_dst : in std_logic_vector(REGS'range); -- The register banc's write address in the EX_MEM pipeline register
                    MEM_ER_er_ctrl_RESG_W : in std_logic;             -- Write signal for the register banc in the EX_MEM pipeline register
                    MEM_ER_reg_dst : in std_logic_vector(REGS'range); -- The register banc's write address in the EX_MEM pipeline register
                    EX_CTRL_ALU_SRCB	: in MUX_ALU_B;	                 -- Control siganl of the ALU SRCB multiplexer (to ensure that rt is used as UAL entries)
                    signal ALU_SEND_SRCA	: out MUX_ALU_A_SEND;		      -- Control siganl of the ALU source multiplexer 
                    signal ALU_SEND_SRCB	: out MUX_ALU_B_SEND	);      -- Control siganl of the ALU source multiplexer
  
  --  Procedure de detection des aleas 
  --  Permet de detecter et resoudre les aleas due a une instruction de chargement   
  --  suivie d'une instruction de type R utilisant la donnee extraite de la memoire
  procedure detection_alea ( EI_DI_OP : in std_logic_vector(OPCODE'length-1 downto 0);  -- Operation code in the DI stage
                             DI_EX_OP : in std_logic_vector(OPCODE'length-1 downto 0);  -- Operation code in the EX stage
                             EI_DI_rs : in std_logic_vector(REGS'range);  -- register source rs in the DI stage (ALU source of instructions)
                             EI_DI_rt : in std_logic_vector(REGS'range);  -- register source rt in the DI stage (ALU source of R instructions)
                             DI_EX_rt : in std_logic_vector(REGS'range);  -- register source rt in the EX stage (dst register of LW instructions)
                             signal EI_CTRL_SEG : out std_logic; 
                             signal DI_CTRL_SEG : out std_logic ); 
end cpu_package;

-- -----------------------------------------------------------------------------
-- the package contains types, constants, and function prototypes
-- -----------------------------------------------------------------------------
package body cpu_package is

-- ===============================================================
-- DEFINITION DE FONCTIONS/PROCEDURES
-- ===============================================================

-- fonction log2
function log2 (I: in natural) return natural is
	variable ip : natural := 1; -- valeur temporaire
	variable iv : natural := 0; -- nb de bits
begin
	while ip < i loop
		ip := ip + ip; -- ou ip := ip * 2
		iv := iv + 1;
	end loop;
	-- renvoie le nombre de bits
	return iv;
end log2;

-- fonction "+" --> procedure adder_cla
function "+" (A,B: in std_logic_vector) return std_logic_vector is
	variable tmp_S : std_logic_vector(A'range);
	variable tmp_COUT,tmp_V : std_logic;
begin
	adder_cla(A,B,'0',tmp_S,tmp_COUT,tmp_V);
	return tmp_S;
end "+";

-- Le drapeau overflow V ne sert que lors d'operations signees !!!
-- Overflow V=1 si operation signee et :
--		addition de deux grands nombres positifs dont le resultat < 0
--		addition de deux grands nombres negatifs dont le resultat >= 0
--		soustraction d'un grand nombre positif et d'un grand nombre negatif dont le resultat < 0
--		soustraction d'un grand nombre negatif et d'un grand nombre positif dont le resultat >= 0
--	Reviens a faire V = C_OUT xor <carry entrante du dernier bit>
-- procedure adder_cla
procedure adder_cla (A,B: in std_logic_vector;C_IN : in std_logic;
						S : out std_logic_vector; C_OUT : out std_logic;
						V : out std_logic) is
	variable G_CLA,P_CLA	: std_logic_vector(A'length-1 downto 0);
	variable C_CLA			: std_logic_vector(A'length downto 0);
begin
	-- calcul de P et G
	G_CLA:= A and B;
	P_CLA:= A or B;
	C_CLA(0):=C_IN;
	for I in 0 to (A'length-1) loop
		C_CLA(I+1):= G_CLA(I) or (P_CLA(I) and C_CLA(I));
	end loop;
	-- mise a jour des sorties
	S:=(A Xor B) xor C_CLA(A'length-1 downto 0);
	C_OUT:=C_CLA(A'length);
	V:= C_CLA(A'length) xor C_CLA(A'length - 1);
end adder_cla;

-- procedure alu
procedure alu (A,B: in std_logic_vector;signal S: out std_logic_vector;
					signal N,V,Z,C: out std_logic;SIGNED_OP: in std_logic;
					CTRL_ALU: in ALU_OPS) is
	variable DATA_WIDTH : positive := A'length;
	variable b_in       : std_logic_vector(DATA_WIDTH-1 downto 0);
	variable c_in       : std_logic;
	variable tmp_S		    : std_logic_vector(DATA_WIDTH-1 downto 0);
	variable tmp_V		    : std_logic;
	variable tmp_N		    : std_logic;
	variable tmp_C		    : std_logic;
	variable tmp_CLA_C	 : std_logic;
	variable tmp_CLA_V	 : std_logic;
begin
	-- raz signaux
	tmp_V := '0';
	tmp_N := '0';
	tmp_C := '0';
	tmp_S := conv_std_logic_vector(0,DATA_WIDTH);
	
	-- case sur le type d'operation
	case CTRL_ALU is
		when ALU_ADD | ALU_SUB | ALU_SLT =>
			b_in := B;
			c_in := '0';
			if (CTRL_ALU /= ALU_ADD) then
				b_in := not(B);
				c_in := '1';
			end if;
			adder_cla(A,b_in,c_in,tmp_S,tmp_C,tmp_V);
			if (CTRL_ALU = ALU_SLT) then
				tmp_S := conv_std_logic_vector( (SIGNED_OP and (tmp_V xor tmp_S(DATA_WIDTH-1))) or (not(SIGNED_OP) and not(tmp_C)) , S'length );
				-- tmp_N := tmp_SLT; Ni N ni V ne doivent etre affectes
			else
				tmp_C := not(SIGNED_OP) and tmp_C;
				tmp_N := SIGNED_OP and tmp_S(DATA_WIDTH-1);
				tmp_V := SIGNED_OP and tmp_V;
			end if;
		when ALU_AND =>
			tmp_S := A and B;
		when ALU_OR =>
			tmp_S := A or B;
		when ALU_NOR =>
			tmp_S := A nor B;
		when ALU_XOR =>
			tmp_S := A xor B;
		when ALU_LSL =>
			tmp_S := shl(A,B);
		when ALU_LSR =>
			tmp_S := shr(A,B);
		when others =>
	end case;
	-- affectation de la sortie
	S <= tmp_S;
	-- affectation du drapeau Z (valable dans tous les cas)
	if (tmp_S=conv_std_logic_vector(0,DATA_WIDTH)) then Z <= '1';
	else Z <= '0';
	end if;
	-- affectation des autres drapeaux N,V,C
	C <= tmp_C;
	N <= tmp_N;
	V <= tmp_V;
end alu;

-- === Procedure control =========================================
--		Permet de positionner les signaux de control pour chaque etage (EX MEM ER)
--		en fonction de l'instruction identifiee soit par son code op, soit par
--		son code fonction, soit par son code branchement.
procedure control ( OP : in std_logic_vector(OPCODE'length-1 downto 0);
							      F  : in std_logic_vector(FCODE'length-1 downto 0);
							      B  : in std_logic_vector(BCODE'length-1 downto 0);
							      b_condition : in std_logic;
							      j_counter   : in std_logic_vector(1 downto 0);
							      di_halt     : in std_logic;
							      signal ei_flush    : out std_logic;
	                  signal di_flush    : out std_logic;
	                  signal ex_flush    : out std_logic;
							      signal j_increment : out std_logic;
							      signal DI_ctrl	    : out mxDI;		-- signaux de controle de l'etage DI
							      signal EX_ctrl	    : out mxEX;		-- signaux de controle de l'etage EX
							      signal MEM_ctrl	   : out mxMEM;	-- signaux de controle de l'etage MEM
							      signal ER_ctrl	    : out mxER ) is	-- signaux de controle de l'etage ER

begin
	-- Initialisation
	DI_ctrl  <= DI_DEFL;
	EX_ctrl  <= EX_DEFL;
	MEM_ctrl <= MEM_DEFL;
	ER_ctrl  <= ER_DEFL;
    ei_flush <= '0';
	di_flush <= '0';
	ex_flush <= '0';
	    
  -- global control to check J or B instruction is alerady fetched and decoded
  -- and if we must halt DI stage
  if ( ( j_counter /= "00" )  or ( b_condition = '1' ) or (di_halt = '1') ) then	
	  if ( b_condition = '1') then
	    ei_flush <= '1';
	    di_flush <= '1';
	    ex_flush <= '1';
	  elsif (j_counter /= "00") then
	   -- increment j_count
	   j_increment <= '1';
	  end if;
	  -- else if di_halt do nothing because signal alerady set to default in initialization step
	
  else	 
    -- increment j_count
	  j_increment <= '0';
	  
    ------------------------------------------------------------------
    -- Signal control of type R operations
    ------------------------------------------------------------------	 
	  if (OP=TYPE_R) then
	  
	    -- Test signed/unsigned operation
	  	 if ((F=ADDU) or (F=SUBU) or (F=SLTU)) then
		    EX_ctrl.ALU_SIGNED <= '0';
		    MEM_ctrl.DC_SIGNED <= '0';
		  else 
		     EX_ctrl.ALU_SIGNED <= '1';
		     MEM_ctrl.DC_SIGNED <= '1';
	    end if; 
	  
	    -- Ex control signals
	    if (F=LSR or F=LSL) then
	      EX_ctrl.ALU_SRCA   <= REGS_QB;
	      EX_ctrl.ALU_SRCB   <= VAL_DEC;
	    else 
	      EX_ctrl.ALU_SRCA   <= REGS_QA;
	      EX_ctrl.ALU_SRCB   <= REGS_QB;
	    end if;
	  
	    EX_ctrl.REG_DST    <= REG_RD;
	  
	    -- ALU opertation signal
	    if ((F=ADD) or (F=ADDU)) then
	      EX_ctrl.ALU_OP <= ALU_ADD;
	    elsif ((F=SUB) or (F=SUBU)) then
	      EX_ctrl.ALU_OP <= ALU_SUB;
	    elsif ((F=SLT) or (F=SLTU)) then
	      EX_ctrl.ALU_OP <= ALU_SLT;
	    elsif (F=iOR) then
	      EX_ctrl.ALU_OP <= ALU_OR;
	    elsif (F=iAND) then
	      EX_ctrl.ALU_OP <= ALU_AND;
	    elsif (F=iNOR) then
	      EX_ctrl.ALU_OP <= ALU_NOR;
	    elsif (F=iXOR) then
	      EX_ctrl.ALU_OP <= ALU_XOR;
	    elsif (F=LSL) then
	      EX_ctrl.ALU_OP <= ALU_LSL;
	    elsif (F=LSR) then
	      EX_ctrl.ALU_OP <= ALU_LSR;
	    end if; 
	  
	    -- MEM signals for the moment (disabled)
	    MEM_ctrl.DC_DS     <= MEM_32;
	    MEM_ctrl.DC_RW     <= '0';
	    MEM_ctrl.DC_AS     <= '0';

	    -- ER control signals
	    ER_ctrl.REGS_W     <= '0';							 -- signal d'ecriture W* du banc de registres
      ER_ctrl.REGS_SRCD	 <= ALU_S;						-- mux vers bus de donnee D du banc de registres
	                 
	  end if;
	
	
    ------------------------------------------------------------------
    -- Signal control of type I operations
    ------------------------------------------------------------------ 

	  -- Operations with an immediat value and two registers (rs source and et register destination)
	  if ((OP=ADDI) or (OP=ADDIU) or (OP=SLTI) or (OP=SLTIU) or (OP=ANDI)
	       or (OP=ORI) or (OP=XORI)) then
	  
	    -- Test signed/unsigned operation
		  if ((F=ADDIU) or (F=SLTIU)) then
		    EX_ctrl.ALU_SIGNED <= '0';
		    MEM_ctrl.DC_SIGNED <= '0';
		  else 
		    EX_ctrl.ALU_SIGNED <= '1';
		    MEM_ctrl.DC_SIGNED <= '1';
	    end if;
	  
	    EX_ctrl.ALU_SRCA   <= REGS_QA;
	    EX_ctrl.ALU_SRCB   <= IMMD;
	    EX_ctrl.REG_DST    <= REG_RT; 
	  
	    -- ALU opertation signal
	    if ((OP=ADDI) or (OP=ADDIU)) then
	      EX_ctrl.ALU_OP <= ALU_ADD;
	    elsif ((OP=SLTI) or (OP=SLTIU)) then
	      EX_ctrl.ALU_OP <= ALU_SLT;
	    elsif (OP=ANDI) then
	      EX_ctrl.ALU_OP <= ALU_AND;
	    elsif (OP=ORI) then
	      EX_ctrl.ALU_OP <= ALU_OR;
	    elsif (OP=XORI) then
	      EX_ctrl.ALU_OP <= ALU_XOR;
	    end if;
	  
	    -- MEM signals for the moment (disabled)
	    MEM_ctrl.DC_DS     <= MEM_32;
	    MEM_ctrl.DC_RW     <= '0'; -- not important (we have not a write or read memory operations)
	    MEM_ctrl.DC_AS     <= '0'; -- data memory not actif

	    -- ER control signals
	    ER_ctrl.REGS_W     <= '0';							 -- signal d'ecriture W* du banc de registres
		  ER_ctrl.REGS_SRCD	 <= ALU_S;						-- the written value in the register banc is the alu result 
	                 
	  end if;
	
	  -- LUI operation control signals
	  if (OP=LUI) then
	    EX_ctrl.ALU_OP     <= ALU_LSL;
	    EX_ctrl.ALU_SIGNED <= '0';
	    EX_ctrl.ALU_SRCA   <= IMMD;
	    EX_ctrl.ALU_SRCB   <= VAL_16; -- register source to compute the destination
	  end if;
	
	  -- Memory load and store operations 
	  if ((OP=LB) or (OP=LH) or (OP=LW) or (OP=LBU) or (OP=LHU)
	       or (OP=SB) or (OP=SH) or (OP=SW)) then
	  
	    -- Test signed/unsigned operation
		  if ((OP=LBU) or (OP=LHU)) then
		    EX_ctrl.ALU_SIGNED <= '0';
		    MEM_ctrl.DC_SIGNED <= '0';
		  else 
		    EX_ctrl.ALU_SIGNED <= '1';
		    MEM_ctrl.DC_SIGNED <= '1';
	    end if;
	  
	    EX_ctrl.ALU_SRCA   <= REGS_QA; -- register source to compute the destination
	    EX_ctrl.ALU_SRCB   <= IMMD;
	    EX_ctrl.REG_DST    <= REG_RT;  -- is important only for load operations  
	    -- ALU opertation signal
	    EX_ctrl.ALU_OP <= ALU_ADD;
	
	    -- MEM signals: Data size signal
	    if ((OP=LB) or (OP=LBU) or (OP=SB)) then
	      MEM_ctrl.DC_DS     <= MEM_8;
	    elsif ((OP=SW) or (OP=LW))  then
	      MEM_ctrl.DC_DS     <= MEM_32;
	    else
	      MEM_ctrl.DC_DS     <= MEM_16;
	    end if;
	  
	    -- MEM signals: Memory Activation signal  
	    MEM_ctrl.DC_AS     <= '1'; -- data memory actif for all operations in this case
	  
	    -- MEM signals: WR signal & ER signals
	    if ((OP=SB) or (OP=SH) or (OP=SW)) then
	      MEM_ctrl.DC_RW   <= '0'; -- for store operations we activate the data cache write signal
	      ER_ctrl.REGS_W   <= '1'; -- no data to be written in register banc
	    else 
	      MEM_ctrl.DC_RW     <= '1'; -- for load operations we activate the data cache read signal
	      ER_ctrl.REGS_W     <= '0'; -- data read from memory will be stored in register bank (W = 0)
	      ER_ctrl.REGS_SRCD	 <= MEM_Q; -- the read data from memory has to be written 
	    end if;
	    	                 
	  end if;
	
	  -- Branch instruction of type I
	  if ((OP=BEQ) or (OP=BNE) or (OP=BLEZ) or (OP=BGTZ)) then  
	    -- not signed operation
  	   DI_ctrl.SIGNED_EXT <= '0';
		  MEM_ctrl.DC_SIGNED <= '0';
		   
	     EX_ctrl.ALU_SRCA   <= REGS_QA;
      	EX_ctrl.ALU_OP     <= ALU_SUB;
      	EX_ctrl.ALU_SIGNED <= '1';
    	
	     if (OP=BEQ) then 
	       EX_ctrl.BRA_SRC <= BRANCHEMENT_BEQ;
	       EX_ctrl.ALU_SRCB   <= REGS_QB;
	     elsif (OP=BNE) then 
	       EX_ctrl.BRA_SRC <= BRANCHEMENT_BNE;
	       EX_ctrl.ALU_SRCB   <= REGS_QB;
	     elsif (OP=BLEZ) then
	       EX_ctrl.BRA_SRC <= BRANCHEMENT_BLEZ;
	       EX_ctrl.ALU_SRCB   <= VAL_ZERO;  
	     elsif (OP=BGTZ) then
	       EX_ctrl.BRA_SRC <= BRANCHEMENT_BGTZ;
	       EX_ctrl.ALU_SRCB   <= VAL_ZERO;
	     end if;
	 
	   end if;
	    
    ------------------------------------------------------------------
    -- Decode of J instruction
    ------------------------------------------------------------------
    if ((OP=J) or (OP=JAL)) then
      -- increment j_count variable/semaphore
      j_increment <= '1';
      
	    -- set signed signal to 0
	    DI_ctrl.SIGNED_EXT <= '0';
		  EX_ctrl.ALU_SIGNED <= '0';
		  MEM_ctrl.DC_SIGNED <= '0';    
		  
	    -- disable write signals
	    MEM_ctrl.DC_RW     <= '0';
	    ER_ctrl.REGS_W     <= '0'; 
	       
	    -- Set saut signal
	    EX_ctrl.SAUT <= '1';   
	     
	    -- if JAL then write NextPC to R31 (ra)
	    if (OP=JAL) then
	       -- We save PC+4 value in R31 regidter
	       EX_ctrl.REG_DST    <= R31;
	     
	       ER_ctrl.REGS_W     <= '0';							 -- signal d'ecriture W* du banc de registres
		     ER_ctrl.REGS_SRCD	 <= NextPC;					-- mux vers bus de donnee D du banc de registres
		   end if;
		 
	  end if;
	 
    ------------------------------------------------------------------
    -- Signal control Instruction de Type B : Branchement
    ------------------------------------------------------------------	 
  	 if (OP=TYPE_B) then
  	   -- not signed operation
  	   DI_ctrl.SIGNED_EXT <= '0';
		  MEM_ctrl.DC_SIGNED <= '0';
		   
	    EX_ctrl.ALU_SRCA   <= REGS_QA;
	    EX_ctrl.ALU_SRCB   <= VAL_ZERO;
  	   EX_ctrl.REG_DST    <= REG_RD;
  	   EX_ctrl.ALU_SIGNED <= '1';
	  
  	   -- MEM signals for the moment (disabled)
	    MEM_ctrl.DC_DS     <= MEM_32;
  	   MEM_ctrl.DC_RW     <= '0';
	    MEM_ctrl.DC_AS     <= '0';

  	   -- ER control signals
	    ER_ctrl.REGS_W     <= '0';							 -- signal d'ecriture W* du banc de registres
    	 ER_ctrl.REGS_SRCD	 <= ALU_S;						-- mux vers bus de donnee D du banc de registres
    	
    	 -- set instruction to SUB
    	 EX_ctrl.ALU_OP <= ALU_SUB;
    	
	    if (B=BLTZ) then -- bltz Rx, offset
	      EX_ctrl.BRA_SRC <= BRANCHEMENT_BLTZ;
	    elsif (B=BGEZ) then -- bgez Rx, offset
	      EX_ctrl.BRA_SRC <= BRANCHEMENT_BGEZ;
	    elsif (B=BLTZAL) then -- bltzal Rx, offset, R31 = PC+4
	      EX_ctrl.BRA_SRC <= BRANCHEMENT_BLTZAL;
	     
	      -- We save PC+4 value in R31 register
	      EX_ctrl.REG_DST    <= R31;
	     
	      ER_ctrl.REGS_W     <= '0';							 -- signal d'ecriture W* du banc de registres
		    ER_ctrl.REGS_SRCD	 <= NextPC;					-- mux vers bus de donnee D du banc de registres
		   
	    elsif (B=BGEZAL) then -- bgezal Rx, offset, R31 = PC+4
	      EX_ctrl.BRA_SRC <= BRANCHEMENT_BGEZAL;
	     
	      -- We save PC+4 value in R31 regidter
	      EX_ctrl.REG_DST    <= R31;
	     
	      ER_ctrl.REGS_W     <= '0';							 -- signal d'ecriture W* du banc de registres
		    ER_ctrl.REGS_SRCD	 <= NextPC;					-- mux vers bus de donnee D du banc de registres
	    end if;
	   
    end if;
  
	end if;
	
end control;

-- === Procedure envoi =========================================
--		Permet de positionner les entrees de l'ALU pour donner la bonne valeur 
--		commme parametre afin de remedier aux aleas de donnes 
procedure envoi ( DI_EX_rs : in std_logic_vector(REGS'range);       -- rs register address in the EX stage
                  DI_EX_rt : in std_logic_vector(REGS'range);       -- rt register address in the EX stage
                  EX_MEM_er_ctrl_RESG_W : in std_logic;             -- Write signal for the register banc in the EX_MEM pipeline register
                  EX_MEM_reg_dst : in std_logic_vector(REGS'range); -- The register banc's write address in the EX_MEM pipeline register
                  MEM_ER_er_ctrl_RESG_W : in std_logic;             -- Write signal for the register banc in the EX_MEM pipeline register
                  MEM_ER_reg_dst : in std_logic_vector(REGS'range); -- The register banc's write address in the EX_MEM pipeline register
                  EX_CTRL_ALU_SRCB	: in MUX_ALU_B;                  -- Control siganl of the ALU SRCB multiplexer (to ensure that rt is used as UAL entries)
                  signal ALU_SEND_SRCA	: out MUX_ALU_A_SEND;		      -- Control siganl of the ALU source multiplexer 
                  signal ALU_SEND_SRCB	: out MUX_ALU_B_SEND	) is    -- Control siganl of the ALU source multiplexer
	               
begin
  
  -- Verify that the source register rs is 0, in this case there is no need to send value (always R0=0) 
  if ( DI_EX_rs /= 0 ) then
    
    -- Test data hazard of EX stage for the rs register
    if ( (EX_MEM_er_ctrl_RESG_W = '0') and (EX_MEM_reg_dst = DI_EX_rs) ) then 
      ALU_SEND_SRCA <= SEND_UNIT_EX;
    
    -- Test data hazard of MEM stage for the rs register
    elsif ( (MEM_ER_er_ctrl_RESG_W = '0') and (MEM_ER_reg_dst = DI_EX_rs)) then 
      ALU_SEND_SRCA <= SEND_UNIT_MEM;
    else
      ALU_SEND_SRCA <= SRC_MUX_ALU_A;
  end if;
  
  -- Testing that the value of rt register is different of 0 and rt is selected as the ALU source parameter
  elsif ( (EX_CTRL_ALU_SRCB = REGS_QB) and (DI_EX_rt /= 0) ) then
  
    --Test data hazard of EX stage for the rt register
    if ( (EX_MEM_ER_ctrl_RESG_W = '0') and (EX_MEM_reg_dst = DI_EX_rt)) then 
      ALU_SEND_SRCB <= SEND_UNIT_EX;
    
    -- Test data hazard of EX stage for the rt register
    elsif ( (MEM_ER_ER_ctrl_RESG_W = '0') and (MEM_ER_reg_dst = DI_EX_rt)) then 
      ALU_SEND_SRCB <= SEND_UNIT_MEM;
    else
      ALU_SEND_SRCB <= SRC_MUX_ALU_B;
    end if;
  end if;  

end envoi;

-- === Procedure de detection des aleas =========================================
--    Permet de detecter et resoudre les aleas due a une instruction de chargement   
--    suivie d'une instruction de type R utilisant la donnee extraite de la memoire
procedure detection_alea ( EI_DI_OP : in std_logic_vector(OPCODE'length-1 downto 0);  -- Operation code in the DI stage
                           DI_EX_OP : in std_logic_vector(OPCODE'length-1 downto 0);  -- Operation code in the EX stage
                           EI_DI_rs : in std_logic_vector(REGS'range);  -- register source rs in the DI stage (ALU source of instructions)
                           EI_DI_rt : in std_logic_vector(REGS'range);  -- register source rt in the DI stage (ALU source of R instructions)
                           DI_EX_rt : in std_logic_vector(REGS'range);  -- register source rt in the EX stage (dst register of LW instructions)
                           signal EI_CTRL_SEG : out std_logic; 
                           signal DI_CTRL_SEG : out std_logic ) is 
begin

  -- Test if we have a load instruction followed by a a R instruction
  -- & Test the use of the same register as source and destination in the R and load instructions
  if ( (EI_DI_OP = TYPE_R) and ((DI_EX_OP=LB) or (DI_EX_OP=LH) or (DI_EX_OP=LW) or (DI_EX_OP=LBU) or (DI_EX_OP=LHU)) 
        and ((EI_DI_rs = DI_EX_rt) or (EI_DI_rt = DI_EX_rt)) ) then    
    EI_CTRL_SEG <= '1';
    DI_CTRL_SEG <= '1';
  else
    EI_CTRL_SEG <= '0';
    DI_CTRL_SEG <= '0';
  end if;           
                         
end detection_alea;

end cpu_package;

