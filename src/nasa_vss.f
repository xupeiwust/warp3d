c
c     ****************************************************************
c     *                                                              *
c     *               NASA-VSS Sparse Symmetric Solver:              *
c     *           Equation package developed by NASA Langley         *
c     *           All key arrays are dynamically allocated using     *
c     *           F-90 features.                                     *
c     *                                                              *
c     *                                                              *
c     *                       written by  : rhd                      *
c     *                       modified by : rh                       *
c     *                                                              *
c     *                   last modified : 02/18/2026 rhd             *
c     *                                                              *
c     ****************************************************************
c
       subroutine direct_sparse_vss( neqns, ncoeff, k_diag, p_vec, 
     &                        u_vec, k_coeffs, k_ptrs, k_indexes,
     &                        cpu_stats, itype, out )
c
      implicit none
c
c                parameter declarations
c
      integer :: neqns, ncoeff, k_ptrs(*), k_indexes(*), itype, out
      double precision :: k_diag(*), p_vec(*), k_coeffs(*), u_vec(*)
      logical :: cpu_stats
c
c                locally defined.
c
      integer, allocatable :: perm(:), xadj(:), adjncy(:), maxan(:),
     &                        xlnz(:), nzsub(:), xnzsub(:), itemp1(:),
     &                        itemp2(:), itemp3(:), itemp4(:)
      integer :: reorder_type, isize1, isize2, maxsub, symb_iters,
     &           lflag, maxlnz, jmax, ncof, maxn, nt  
      double precision, allocatable :: s_matrix(:), rtemp1(:)
      save :: perm, xadj, adjncy, xlnz, nzsub, xnzsub, maxsub, maxlnz 
      real, external :: wwalltime
c
c
c                solution types (itype):
c                 1 - first time solution for a matrix
c                 2 - re-solution of same matrix equations
c                     with a new set of coefficients but same
c                     sparsity
c                 3 - no solution. just release saved, allocated
c                     arrays used to support solution type 2.
c
c		1.-  Release allocated Arrays
c
c
      if ( itype .eq. 3 ) then
       if ( allocated ( perm           ) ) deallocate( perm      )
       if ( allocated ( xadj           ) ) deallocate( xadj      )
       if ( allocated ( adjncy         ) ) deallocate( adjncy    )
       if ( allocated ( xlnz           ) ) deallocate( xlnz      )
       if ( allocated ( nzsub          ) ) deallocate( nzsub     )
       if ( allocated ( xnzsub         ) ) deallocate( xnzsub    )
       return
      end if
c
c
c		2.- Perform symbolic re-ordering process followed
c                   by some consistent size checks. for re-solve with
c                   same sparsity, much of the re-ordering work is skipped.
c                 reorder_type:  1 never solved before or sparsity new
c                                2 same sparsity, different coeffs.
c    
!      call thyme( 23, 1 )
      reorder_type = 2
      if ( itype .eq. 1 ) then
        if ( allocated( perm )   ) deallocate( perm )
        if ( allocated( xadj )   ) deallocate( xadj )
        if ( allocated( adjncy ) ) deallocate( adjncy )
        isize1       = neqns + 1
        isize2       = ncoeff*2 + neqns
        allocate( perm(isize1) )
        allocate( xadj(isize1) )
        allocate( adjncy(isize2) )
        reorder_type = 1
      end if
      call sparse_reorder( neqns, ncoeff, k_diag, p_vec, k_coeffs,
     &                     k_ptrs, k_indexes, perm, xadj, adjncy,
     &                     reorder_type )
      if ( cpu_stats ) write(out,9482) wwalltime(1)
!      call thyme( 23, 2 )
c
      if ( reorder_type .eq. 2 ) go to 800
c
!      call thyme( 24, 1 )
      allocate( maxan(neqns+1) )
      call sparse_check( neqns, k_ptrs, k_indexes, maxan, ncof, maxn )
      call sparse_adj( neqns, k_ptrs, k_indexes, ncof,
     &                 maxn, nt, xadj, adjncy, jmax, maxan )
      deallocate( maxan )
      if ( cpu_stats ) write(out,9484) wwalltime(1)
c
c            2b.- perform symbolic factorization to compute fill-in
c                 during solution. lflag indicates if current memory
c                 allocation will be sufficient for actual solution.
c                 if not, resize and try again. re-size matrices if
c                 required due to change in number of equations.
c
      maxsub = 2 * ncoeff
      symb_iters = 0
      if ( allocated(xlnz)   ) deallocate( xlnz ) 
      if ( allocated(nzsub)  ) deallocate( nzsub ) 
      if ( allocated(xnzsub) ) deallocate( xnzsub ) 
      allocate( xlnz(neqns+1) )
      allocate( xnzsub(neqns+1) )
 700  continue          
      maxsub = int(1.2 * real(maxsub))
      allocate( nzsub(maxsub) )
      call sparse_sym_fact( neqns, xadj, adjncy, xlnz, maxlnz, xnzsub,
     &                      nzsub, maxsub, lflag )
      if( lflag .eq. 0 ) then
         if ( cpu_stats ) then
            write(out,9161) wwalltime(1)
            write(out,9171) maxsub
            write(out,9181) maxlnz
         end if
!         call thyme( 24, 2 )
         go to 800
      else
         symb_iters = symb_iters + 1
         write(out,9191) 
         if ( symb_iters .eq. 3 ) then
              write(out,9201) maxsub
             stop
         end if
         deallocate( nzsub ) 
      end if
      go to 700
c
c            3.-  re-ordering and symbolic factorization completed
c                 satisfactorily. s_matrix is the numerically factorized
c                 matrix. fill it with non-zero terms of coefficients.
c                 we can then release the compacted coefficients and
c                 k pointers/indexes.
c
800   continue
c   
!      call thyme( 25, 1 )
      allocate( s_matrix(maxlnz) )
      call sparse_fill( neqns, k_indexes, xlnz, maxlnz, k_coeffs,
     &                  s_matrix, k_ptrs, nzsub, xnzsub )
      if ( cpu_stats ) write(out,9486) wwalltime(1)
c
c            4.-  numeric factorization, forward pass, back pass.
c                 p_vec holds displacements on return but in 
c                 re-ordered form. 
c
c
      allocate( itemp1(neqns+1) )
      allocate( itemp2(neqns+1) )
      allocate( itemp3(neqns+1) )
      allocate( itemp4(neqns+1) )
      allocate( rtemp1(neqns+1) )
      call sparse_factor( neqns, xlnz, s_matrix,
     &                    k_diag, nzsub, xnzsub, itemp1, itemp2,
     &                    itemp3, itemp4, rtemp1 )
      deallocate( itemp1 )
      deallocate( itemp2 )
      deallocate( itemp3 )
      deallocate( itemp4 )
      deallocate( rtemp1 )
      if ( cpu_stats ) write(out,9490) wwalltime(1)
!      call thyme( 25, 2 )
!      call thyme( 26, 1 )
c
c            5.- Solve: forward and backword substitutions
c
      call sparse_loadpass( neqns, xlnz, maxlnz, s_matrix, k_diag,
     &                      p_vec, nzsub, xnzsub  )
      deallocate( s_matrix )
c
      u_vec(perm(1:neqns)) = p_vec(1:neqns)
      if ( cpu_stats ) write(out,9492) wwalltime(1)
!      call thyme( 26, 2 )
c
      return
c
 9482  format(
     &  15x, 'VSS sparse reordering done    @ ',f10.2 )
 9484  format(
     &  15x, 'sparse checking done          @ ',f10.2 )
 9486  format(
     &  15x, 'sparse matrix loaded          @ ',f10.2 )
 9490  format(
     &  15x, 'numeric factorization done    @ ',f10.2 )
 9492  format(
     &  15x, 'numeric loadpass done         @ ',f10.2,/ )
c
 9161  format(
     &  15x, 'symbolic factorization done   @ ',f10.2)
 9171  format(
     &  15x, 'memory for nzsub (sp words)     ',i10)
 9181  format(
     &  15x, 'no. terms in factored matrix:   ',i10)
 9191  format(
     &  15x, 're-sizing work for sym. factor  ')
 9201  format(1x,
     &'>> FATAL ERROR: Job Aborted.',
     &5x,'increasing length of work space for symbolic factorization',
     &/5x,'three times to maxsub:',i12,' not sufficient')
c
      end
c ------------------------------------------------------------------------
c
c                 NASA-VSS sparse matrix storage format
c
c   example:
c
c                1    2    3    4     5     6
c
c          1  | 100   1    2                5  |  | x1 |     | 201 |
c          2  |     200    6    7           9  |  | x2 |     | 202 |
c          3  |          300   10    11    12  |  | x3 |     | 203 |
c      a = 4  |                400   13    14  |  | x4 |  =  | 204 |
c          5  |                     500    15  |  | x5 |     | 205 |
c          6  |                           600  |  | x6 |     | 206 |
c
c     number of equations    = 6
c
c     number of coefficients = 12
c
c
c     k_ptrs    = { 3, 3, 3, 2, 1, 0}
c
c     k_indesxs = { 2, 3, 6,  3, 4, 6,  4, 5, 6,  5, 6,  6}
c
c     k_coefs   = { 1, 2, 5,  6, 7, 9, 10,11,12, 13,14, 15}
c
c     k_diag    = { 100, 200, 300, 400, 500, 600}
c
c     k_rhs     = { 201, 202, 203, 204, 205, 206}
c
c ------------------------------------------------------------------------
