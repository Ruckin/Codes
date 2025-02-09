-- 015 VHDL CODE
-- FREDERICO ANTONIAZZI - 01/07/2024
--
-- DEVELOPEMENT OF A STATE MACHINE IN VHDL
-- MOORE MACHINE
--

-- CREATING THE ENTITY
entity StateMachine is port
(

clk : in bit; -- CLOCK INPUT
rst : in bit; -- RESET INPUT

Q : inout bit_vector(1 downto 0) -- IN/OUTPUT VECTOR OF 2 POSITIONS

);
end StateMachine; -- ENDING ENTITY

-- CREATING THE ARCHITECTURE THAT IMPLEMENTS A MOORE STATE MACHINE
architecture Hardware of StateMachine is
begin

	myStateMachine : process(clk, rst)
	
	begin
	
		if    (rst = '1') then Q <= "00";
		elsif (clk'event AND clk = '1') then
			case Q is
				
				when "00" => Q <= "01";
				when "01" => Q <= "11";
				when "11" => Q <= "10";
				when "10" => Q <= "00";
				
			end case;
		end if;
	end process myStateMachine;
end Hardware; -- ENDING ARCHITECTURE
