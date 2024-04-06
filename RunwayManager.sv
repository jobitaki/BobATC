`default_nettype none

import BobATC::*;

//
// Module 'RunwayManager'
// 
// - When lock is asserted, the runway indicated by the runway_id input
//   will be locked and associated with the plane_id input. 
// - When unlock is asserted, the runway indicated by the runway_id input
//   will be unlocked only if the plane_id on the input is equal. 
//
module RunwayManager
  (input  logic       clock, reset_n,
   input  logic [3:0] plane_id,
   input  logic       runway_id,
   input  logic       lock,
   input  logic       unlock,
   output logic [1:0] runway_active);

  // Register that contain the current status of each runway
  // Contains 4 bits of plane ID followed by runway status (1 for lock)
  runway_t [1:0] runway;

  assign runway_active[0] = runway[0].active;
  assign runway_active[1] = runway[1].active;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      runway_active <= 0;
      runway <= 0;
    end else begin
      if (lock && !unlock) begin
        if (runway_id) begin
          runway[1].plane_id <= plane_id;
          runway[1].active <= 1'b1;
        end else begin
          runway[0].plane_id <= plane_id;
          runway[0].active <= 1'b1;
        end
      end else if (!lock && unlock) begin
      end else if (lock && unlock) begin
      end
    end
  end

endmodule : RunwayLock