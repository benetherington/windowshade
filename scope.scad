regular  = "regular global";
$special = "special global";
module show() echo("         in show    ", regular,"   ", $special );

echo ("         outside    ", regular,"   ", $special );
          // ECHO: "         outside    ", "regular global", "   ", "special global"
  
for ( regular = [0:1] ){ echo("in regular loop     ", regular,"   ", $special ); show();}
          // ECHO: "in regular loop     ", 0, "   ", "special global"
          // ECHO: "         in show    ", "regular global", "   ", "special global"
          // ECHO: "in regular loop     ", 1, "   ", "special global"
          // ECHO: "         in show    ", "regular global", "   ", "special global"

for ( $special = [5:6] ){ echo("in special loop     ", regular,"   ", $special ); show();}
          // ECHO: "in special loop     ", "regular global", "   ", 5
          // ECHO: "         in show    ", "regular global", "   ", 5
          // ECHO: "in special loop     ", "regular global", "   ", 6
          // ECHO: "         in show    ", "regular global", "   ", 6

show();
          // ECHO: "         in show    ", "regular global", "   ", "special global"