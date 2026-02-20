c****************************************************************
c
      subroutine  sparse_fill( neqns, adjncy, xlnz, maxlnz,
     &                         amat, s_matrix, irow, iloc, inum )
c
c****************************************************************
c
      implicit none
c      
      integer, intent(in) :: neqns, adjncy(*), xlnz(*), irow(*),
     &                       iloc(*), inum(*), maxlnz
      double precision, intent(in) :: amat(*)
      double precision, intent(inout) :: s_matrix(maxlnz)
c
c              locals
c
      integer :: icont, i, ixx, kcont, jcont, j
      double precision, parameter :: zero = 0.0d0
c
      icont = 1
      do i = 1, maxlnz
       s_matrix(i) = zero
      end do
c
      do i = 1, neqns-1
        if( irow(i) .eq. 0 ) go to 101
        ixx = irow(i)
        kcont = 0
        jcont = inum(i)
        do j = xlnz(i), xlnz(i+1) - 1
              if( iloc(jcont) .eq. adjncy(icont) ) then 
                s_matrix(j) = amat(icont)
                icont = icont + 1
                kcont = kcont + 1
                if( kcont .eq. ixx ) go to 101
              end if
              jcont = jcont + 1
         end do
101      continue
         jcont = jcont + 1
      end do
c
      return
      end
