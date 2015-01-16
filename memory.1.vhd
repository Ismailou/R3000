------------------------------------
-- Banc Memoire pour processeur RISC
-- THIEBOLT Francois le 01/12/05
------------------------------------

---------------------------------------------------------
-- Lors de la phase RESET, permet la lecture d'un fichier
-- passe en parametre generique.
---------------------------------------------------------

------------------------------------------------------------------
-- Ne s'agissant pas encore d'un cache, le signal Ready est cable 
-- a 1 puisque toute operation s'execute en un seul cycle.
--	Ceci est la version avec lecture ASYNCHRONE pour une
--	integration plus simple dans le pipeline.
-- Si la lecture du fichier d'initialisation ne couvre pas tous
--	les mots memoire, ceux-ci seront initialises a 0
------------------------------------------------------------------

-- Definition des librairies
library IEEE;
library STD;
library WORK;

-- Definition des portee d'utilisation
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;
use IEEE.std_logic_textio.all;
use STD.textio.all;
use WORK.cpu_package.all;

-- Definition de l'entite
entity memory is

	-- definition des parametres generiques
	generic	(
		-- largeur du bus de donnees par defaut
		DBUS_WIDTH : natural := 32;

		-- largeur du bus adr par defaut
		ABUS_WIDTH : natural := 32;

		-- nombre d'elements dans le cache exprime en nombre de mots
		MEM_SIZE : natural := 16;

		-- front actif par defaut
		ACTIVE_FRONT : std_logic := '1';

		-- fichier d'initialisation
		FILENAME : string := "" );

	-- definition des entrees/sorties
	port 	(
		-- signaux de controle du cache
		RST			  : in std_logic;	-- actifs a l'etat bas
		CLK,RW		: in std_logic;	-- R/W*
		DS				: in MEM_DS;		-- acces octet, demi-mot, mot, deux mots.
		Signed		: in std_logic;	-- extension de signe
		AS				: in std_logic;	-- Address Strobe (sorte de CS*)
		Ready			: out std_logic;	-- indicateur HIT/MISS
		Berr			 : out std_logic;	-- bus error (acces non aligne par exemple), active low

		-- bus d'adresse du cache
		ADR			  : in std_logic_vector(ABUS_WIDTH-1 downto 0);

		-- Ports entree/sortie du cache
		D				: in std_logic_vector(DBUS_WIDTH-1 downto 0);
		Q				: out std_logic_vector(DBUS_WIDTH-1 downto 0) );

end memory;

-- Definition de l'architecture du banc de registres
architecture behavior of memory is

	-- definition de constantes
	constant BITS_FOR_BYTES : natural := 8 ; -- nb bits adr pour acceder aux octets d'un mot
	constant BITS_FOR_WORDS : natural := 32 ; -- nb bits adr pour acceder aux mots du cache
	constant BYTES_PER_WORD : natural := 4 ; -- nombre d'octets par mot

	-- definitions de types (index type default is integer)
	subtype BYTE is std_logic_vector(BITS_FOR_BYTES-1 downto 0); -- definition d'un octet
	type WORD is array (BYTES_PER_WORD-1 downto 0) of BYTE; -- definition d'un mot composé d'octets

	type FILE_REGS is array (MEM_SIZE-1 downto 0) of WORD;
	subtype I_ADR is std_logic_vector(log2(MEM_SIZE)+2-1 downto 2); -- internal ADR au format mot du cache

	subtype B_ADR is std_logic_vector(1 downto 0); -- byte ADR pour manipuler les octets dans le mot
	subtype byte_adr is natural range 0 to 1; -- manipulation d'octets dans les mots

	-- definition de la fonction de chargement d'un fichier
	--		on peut egalement mettre cette boucle dans le process qui fait les ecritures
	function LOAD_FILE (F : in string) return FILE_REGS is
		variable temp_REGS : FILE_REGS;
		file mon_fichier : TEXT open READ_MODE is STRING'(F); -- VHDL93 compliant
		--	file mon_fichier : TEXT is in STRING'(F); -- older implementation
		variable line_read : line := null;
		variable line_value : std_logic_vector (DBUS_WIDTH-1 downto 0);
		variable index,i : natural := 0;
	begin
		-- lecture du fichier
		index:=0;
		while (not ENDFILE(mon_fichier) and (index < MEM_SIZE))
		loop
			readline(mon_fichier,line_read);
			read(line_read,line_value);
			for i in 0 to BYTES_PER_WORD-1 loop
				temp_REGS(index)(i):=line_value(((i+1)*8)-1 downto i*8);
			end loop;
--			temp_REGS(index):=line_value;
			index:=index+1;
		end loop;
		-- test si index a bien parcouru toute la memoire
--		if (index < MEM_SIZE) then
--			temp_REGS(index to MEM_SIZE-1):=(others => (others => (others => '0')));
--		end if;
		-- renvoi du resultat
		return temp_REGS;
	end LOAD_FILE;

	-- definition des ressources internes
	signal REGS : FILE_REGS; -- le banc memoire

	-- l'adressage de la memoire se faisant par element de taile DBUS_WIDTH, par rapport
	-- au bus d'adresse au format octet il faut enlever les bits d'adresse de poids faible
	-- (octets dans le mot), puis prendre les bits utiles servant a l'acces des mots du cache.
	-- ex.: mots de 32 bits => 2 bits de poids faible pour les octets dans le mot
	--		16 mots memoire => 4 bits necessaire
	-- D'ou I_ADR = ADR (4+2-1 downto 2)
	
begin
------------------------------------------------------------------
-- Affectations dans le domaine combinatoire
-- 

-- Indicateur acces MISS/HIT
Ready <= '1'; -- car pas encore un cache

------------------------------------------------------------------
-- Process P_CACHE
--	La lecture etant asynchrone c.a.d qu'elle ne depend que des
--		signaux d'entree, nous sommes obliges de les mettre dans la
--		liste de sensitivite du process
P_CACHE: process(CLK,RST,ADR,AS,RW,DS,Signed)
  variable i_adr : I_ADR ;
  variable b_adr : B_ADR ;
  variable addr_aligned : std_logic;
  variable tmp_Q : std_logic_vector(31 downto 0) := to_stdlogicvector(BIT_VECTOR'(X"FFFFFFFF"));
begin  
  -- test du front actif d'horloge
	if (CLK'event and CLK=ACTIVE_FRONT) then -- rising_edge(CLK)
    -- When RST
    if ( RST = not(ACTIVE_FRONT)) then
      -- RAZ
      Q <= (others => 'Z');
      -- LOAD file or reset to memory to '0'
      if ( STRING'(FILENAME) /= "") then
        REGS <= LOAD_FILE(FILENAME);
      else
        REGS <= (others => (others => (others => '0')));
      end if;
    end if;
        
    -- Check alignement (DS)
      -- extract I_ADR
      i_adr := ADR(I_ADR'range);
      b_adr := ADR(B_ADR'range); 
      case(DS) is
        when  MEM_8 =>
          -- any @ match
          Berr <= '1';
          addr_aligned := '1';
        when  MEM_16 =>
          -- only three offset match
          if ( b_adr = "00" or b_adr = "01" or b_adr = "10") then
            Berr <= '1';
            addr_aligned := '1';
          else
            Berr <= '0';
            addr_aligned := '0';
          end if;
        when  MEM_32 | MEM_64 =>
          -- only one case is possible
          if ( b_adr = "00") then
            Berr <= '1';
            addr_aligned := '1';
          else
            Berr <= '0';
            addr_aligned := '0';
          end if;       
        when others =>
          addr_aligned := '0';
	   end case;
	   
	   
    -- When WRITE
    if (AS='1' and addr_aligned='1') then
			 -- When Write
			if (RW='0') then
			  case(DS) is
          when  MEM_8 =>
            REGS(conv_integer(i_adr))(conv_integer(b_adr)) <= D(7 downto 0);
          when  MEM_16 =>
            REGS(conv_integer(i_adr))(conv_integer(b_adr)) <= D(7 downto 0);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+1) <= D(15 downto 8);
          when  MEM_32 =>
            REGS(conv_integer(i_adr))(conv_integer(b_adr)) <= D(7 downto 0);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+1) <= D(15 downto 8);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+2) <= D(23 downto 16);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+3) <= D(31 downto 24);
          when  MEM_64 =>
            REGS(conv_integer(i_adr))(conv_integer(b_adr)) <= D(7 downto 0);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+1) <= D(15 downto 8);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+2) <= D(23 downto 16);
            REGS(conv_integer(i_adr))(conv_integer(b_adr)+3) <= D(31 downto 24);
            
            REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)) <= D(7 downto 0);
            REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)+1) <= D(15 downto 8);
            REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)+2) <= D(23 downto 16);
            REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)+3) <= D(31 downto 24);
          when others =>
        end case;
			else -- When READ
				case(DS) is
          when  MEM_8 =>
            tmp_Q(7 downto 0) := REGS(conv_integer(i_adr))(conv_integer(b_adr));
            if (Signed = '0') then
            tmp_Q(31 downto 8) := to_stdlogicvector(BIT_VECTOR'(X"000000"));
            else
              tmp_Q(31 downto 8) := to_stdlogicvector(BIT_VECTOR'(X"FFFFFF"));
            end if;
          when  MEM_16 =>
            tmp_Q(7 downto 0) := REGS(conv_integer(i_adr))(conv_integer(b_adr));
            tmp_Q(15 downto 8) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+1);
            if (Signed = '0') then 
              tmp_Q(31 downto 16) := to_stdlogicvector(BIT_VECTOR'(X"0000"));
            else
              tmp_Q(31 downto 16) := to_stdlogicvector(BIT_VECTOR'(X"FFFF"));
            end if;
          when  MEM_32 =>
            tmp_Q(7 downto 0) := REGS(conv_integer(i_adr))(conv_integer(b_adr));
            tmp_Q(15 downto 8) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+1);
            tmp_Q(23 downto 16) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+2);
            tmp_Q(31 downto 24) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+3);
          when  MEM_64 =>
            tmp_Q(7 downto 0) := REGS(conv_integer(i_adr))(conv_integer(b_adr));
            tmp_Q(15 downto 8) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+1);
            tmp_Q(23 downto 16) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+2);
            tmp_Q(31 downto 24) := REGS(conv_integer(i_adr))(conv_integer(b_adr)+3);
            
            tmp_Q(7 downto 0) := REGS(conv_integer(i_adr)+1)(conv_integer(b_adr));
            tmp_Q(15 downto 8) := REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)+1);
            tmp_Q(23 downto 16) := REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)+2);
            tmp_Q(31 downto 24) := REGS(conv_integer(i_adr)+1)(conv_integer(b_adr)+3);
          when others =>
            tmp_Q := (others => 'Z');
        end case;
        Q <= tmp_Q;
			end if;
		else
		  Berr <= '1';
    end if;
    
  end if;    
end process P_CACHE;

end behavior;

